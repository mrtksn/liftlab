"""Exercise unloaded GPIOs; never run while connected to motors or other targets."""
import argparse
import serial,time,json,threading,pathlib
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port',default='/dev/cu.usbserial-0001')
parser.add_argument('--unloaded',action='store_true',help='Confirm only USB power is connected; this test drives GPIO18/19/21/22/23/25/26/27')
parser.add_argument('--output-dir',type=pathlib.Path,default=pathlib.Path('/tmp/signal-workbench-check'))
args=parser.parse_args()
if not args.unloaded:parser.error('Disconnect targets and motor power, then pass --unloaded to run output tests.')
port=serial.Serial(port=None,baudrate=921600,timeout=.1);port.dtr=False;port.rts=False;port.port=args.port;port.open();time.sleep(1)
lock=threading.Lock();ping=True;counter=0;transcript=[]
def write(msg):
 with lock:port.write((json.dumps(msg,separators=(',',':'))+'\n').encode())
def keepalive():
 while ping:
  write({'cmd':'ping'});time.sleep(.15)
thread=threading.Thread(target=keepalive,daemon=True);thread.start()
def receive(deadline):
 while time.monotonic()<deadline:
  line=port.readline().decode(errors='replace').strip()
  if '@SB ' not in line:continue
  try:obj=json.loads(line.split('@SB ',1)[1])
  except ValueError:continue
  transcript.append(obj);return obj
 raise TimeoutError('No firmware reply')
def command(cmd,expected_error=False,**args):
 global counter
 counter+=1;ident=counter;write(dict(cmd=cmd,id=ident,**args));deadline=time.monotonic()+15;cap=None
 while True:
  obj=receive(deadline)
  if obj.get('id')!=ident:continue
  if obj['type']=='capture':cap=dict(obj,edges=[])
  elif obj['type']=='edges':cap['edges']+=obj['edges']
  elif obj['type']=='captureEnd':assert len(cap['edges'])==cap['count'];return cap
  elif obj['type']=='analog':cap=dict(obj,points=[])
  elif obj['type']=='analogPoints':cap['points']+=obj['points']
  elif obj['type']=='analogEnd':assert len(cap['points'])==cap['count'];return cap
  elif obj['type'] in ('ack','hello','error'):
   assert (obj['type']=='error')==expected_error,obj
   return obj

def capture(ms=60,**args):return command('capture',ms=ms,includeOutputs=True,**args)
def stats(cap,ch=0):return next(a for a in cap['analysis'] if a['ch']==ch)
def expect_pwm(cap,ch,hz,high,tolerance=3):
 a=stats(cap,ch);assert abs(a['hz']-hz)<max(2,hz*.03),(a,hz);assert abs(a['high']-high)<max(tolerance,high*.02),(a,high)
 return {'hz':a['hz'],'high':a['high'],'sampleUs':cap['sampleUs']}
results={}
try:
 hello=command('hello');assert hello['protocol']==2 and hello['adc'];results['hello']=hello
 ch=[{'name':'Output','pin':18,'direction':'out','idle':{'kind':'pwm','hz':490,'value':550}},{'name':'Probe','pin':19,'direction':'in'}]
 command('configure',channels=ch);assert capture(ms=20)['count']==0;command('enable');assert capture(ms=20)['count']==0;command('stop');assert capture(ms=20)['count']==0;results['quietEnableAndUntouchedStop']=True;command('enable');command('signal',ch=0,signal={'kind':'pwm','hz':490,'value':1500},mode='continuous')
 results['pwm']=expect_pwm(capture(triggerCh=0,triggerEdge=1),0,490,1500)
 command('signal',ch=0,signal={'kind':'pwm','hz':490,'value':1200},mode='continuous');results['liveUpdate']=expect_pwm(capture(),0,490,1200)
 command('stop');results['idleAfterStop']=expect_pwm(capture(),0,490,550)
 command('signal',expected_error=True,ch=0,signal={'kind':'digital','value':1},mode='continuous')
 pins=[18,19,21,22,23,25,26,27];freqs=[50,100,200,300,400,490,1000,8000]
 multi=[{'pin':p,'direction':'out','idle':{'kind':'digital','value':0}}for p in pins]
 command('configure',channels=multi);command('enable')
 for i,hz in enumerate(freqs):command('signal',ch=i,signal={'kind':'pwm','hz':hz,'value':250000/hz},mode='continuous')
 cap=capture();results['eightIndependentOutputs']=[expect_pwm(cap,i,hz,250000/hz)for i,hz in enumerate(freqs)]
 command('configure',expected_error=True,channels=[{'pin':34,'direction':'out'}]);command('configure',expected_error=True,channels=[{'pin':18,'direction':'in'},{'pin':18,'direction':'out'}]);results['invalidConfigPreservesOutputs']=expect_pwm(capture(),5,490,250000/490)
 command('stop');command('configure',channels=[{'pin':18,'direction':'out','idle':{'kind':'digital','value':0}}]);command('enable')
 results['dshot']={}
 for rate in [150,300,600]:
  command('signal',ch=0,signal={'kind':'dshot','value':100,'rate':rate},mode='continuous');cap=capture(ms=20);a=stats(cap);assert 'DShot' in a['kind'] or 'DShot' in a['description'],a;results['dshot'][rate]=a
 command('signal',ch=0,signal={'kind':'uart','baud':115200,'data':'55AA'},mode='continuous',repeatMs=5);cap=capture(ms=60);results['uart115200']=stats(cap);assert cap['count']>30
 command('signal',ch=0,signal={'kind':'pulse','pulses':[[1,500],[0,1500]]},mode='continuous',repeatMs=2);results['rawPulseTrain']=expect_pwm(capture(),0,500,500)
 command('signal',ch=0,signal={'kind':'digital','value':1},mode='continuous')
 counter+=1;cid=counter;write({'cmd':'capture','id':cid,'ms':100,'includeOutputs':True});time.sleep(.02)
 counter+=1;sid=counter;write({'cmd':'stop','id':sid});cap=None;done=False;stopped=False;deadline=time.monotonic()+10
 while not(done and stopped):
  obj=receive(deadline)
  if obj.get('id')==sid and obj['type']=='ack':stopped=True
  if obj.get('id')==cid:
   if obj['type']=='capture':cap=dict(obj,edges=[])
   elif obj['type']=='edges':cap['edges']+=obj['edges']
   elif obj['type']=='captureEnd':done=True
 falling=[t for t,v in cap['edges'] if not(v&1)];assert cap['initial']&1 and falling and falling[0]<80000,cap
 results['urgentStopDuringCaptureUs']=falling[0]
 command('enable')
 command('signal',ch=0,signal={'kind':'pulse','pulses':[[1,50000],[0,1000]]},mode='once');cap=capture(ms=100);assert cap['count']>=1;time.sleep(.06);cap=capture(ms=20);assert stats(cap)['edges']==0;results['singleWaveformStops']=True
 command('signal',ch=0,signal={'kind':'digital','value':1},mode='continuous');ping=False;thread.join();time.sleep(1.1)
 command('signal',expected_error=True,ch=0,signal={'kind':'digital','value':1},mode='continuous');cap=capture(ms=20);assert cap['initial']&1==0;assert any(x['type']=='timeout'for x in transcript);results['hostLeaseStopsOutput']=True
 command('stop');command('configure',channels=[{'pin':19,'direction':'in'}]);cap=command('capture',ms=5,triggerCh=0,triggerEdge=1);results['triggerReport']=cap['triggered']
 command('configure',channels=[{'pin':p,'direction':'analog','attenuation':12}for p in [32,33,34,35,36,39,4,13]])
 adc=command('analog',hz=100,ms=500);assert len(adc['points'])==50
 assert all(len(row)==17 and all(0<=row[1+2*j]<=4095 for j in range(8))for row in adc['points'])
 results['adcEightChannels']={k:adc[k]for k in ['count','calibration','channels','duration']}
 command('configure',expected_error=True,channels=[{'pin':18,'direction':'analog'}]);command('analog',expected_error=True,hz=1000,ms=5000)
 print(json.dumps(results,indent=2))
finally:
 ping=False
 try:write({'cmd':'stop','id':9999});time.sleep(.1)
 except Exception:pass
 port.close();path=args.output_dir;path.mkdir(parents=True,exist_ok=True);(path/'verification.json').write_text(json.dumps(results,indent=2));(path/'protocol-transcript.json').write_text(json.dumps(transcript,indent=2))
