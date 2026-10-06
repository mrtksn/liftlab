'use strict';
// Tab content belongs here; appearance and interactions belong to UI.tabs.
const EDITOR_TABS = [
  {
    key: "air",
    id: "tabAir",
    panel: "paneAir",
    label: "Airframe",
    icon: `<svg viewBox="0 0 20 20" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><circle cx="5" cy="5" r="2.4"/><circle cx="15" cy="5" r="2.4"/><circle cx="5" cy="15" r="2.4"/><circle cx="15" cy="15" r="2.4"/><path d="M6.8 6.8l2 2m4.4-2-2 2m-4.4 4.4 2-2m4.4 2-2-2"/></svg>`,
  },
  {
    key: "form",
    id: "tabForm",
    panel: "paneForm",
    label: "Computers",
    icon: `<svg viewBox="0 0 20 20" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><rect x="5" y="5" width="10" height="10" rx="1.5"/><path d="M8 2.5V5m4-2.5V5M8 15v2.5m4-2.5v2.5M2.5 8H5m-2.5 4H5m10-4h2.5M15 12h2.5"/></svg>`,
    badge: {"id": "editedCount"},
  },
  {
    key: "gs",
    id: "tabGs",
    panel: "paneGs",
    label: "Ground",
    icon: `<svg viewBox="0 0 20 20" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><path d="M10 11v7M7 18h6"/><circle cx="10" cy="9" r="1.6"/><path d="M6.5 5.5a5 5 0 0 0 0 7M13.5 5.5a5 5 0 0 1 0 7M4 3a8.5 8.5 0 0 0 0 12M16 3a8.5 8.5 0 0 1 0 12"/></svg>`,
    title: "Ground station: what comes down the radio",
    "aria-label": "Ground station",
  },
  {
    key: "ai",
    id: "tabAi",
    panel: "paneAi",
    label: "AI",
    icon: `<svg viewBox="0 0 20 20" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><path d="M10 2.5 11.6 8.4 17.5 10l-5.9 1.6L10 17.5l-1.6-5.9L2.5 10l5.9-1.6z"/></svg>`,
    title: "An AI agent that can fly, build and tune the simulated drone",
    "aria-label": "AI agent",
  },
];
const READOUT_TABS = [
  {
    key: "flight",
    id: "rtab-flight",
    panel: "rpane-flight",
    label: "Flight",
    icon: `<svg viewBox="0 0 20 20" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><path d="M2 11h3l2-5 3 9 2.5-6 1.5 2H18"/></svg>`,
    title: "What it does now: the actuators, the traces, the cargo",
  },
  {
    key: "health",
    id: "rtab-health",
    panel: "rpane-health",
    label: "Health",
    icon: `<svg viewBox="0 0 20 20" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><path d="M10 16.5s-6.5-3.9-6.5-8.4A3.4 3.4 0 0 1 10 6a3.4 3.4 0 0 1 6.5 2.1c0 4.5-6.5 8.4-6.5 8.4z"/></svg>`,
    title: "The parts, the supervisor and the state estimate",
  },
  {
    key: "control",
    id: "rtab-control",
    panel: "rpane-control",
    label: "Control",
    icon: `<svg viewBox="0 0 20 20" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><path d="M4 4v12M10 4v12M16 4v12"/><circle cx="4" cy="12" r="1.8" fill="currentColor"/><circle cx="10" cy="7" r="1.8" fill="currentColor"/><circle cx="16" cy="11" r="1.8" fill="currentColor"/></svg>`,
    title: "Learning, allocation, headroom and mass",
  },
  {
    key: "world",
    id: "rtab-world",
    panel: "rpane-world",
    label: "World",
    icon: `<svg viewBox="0 0 20 20" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><circle cx="10" cy="10" r="7"/><path d="M3 10h14M10 3c2 2.2 2.8 4.5 2.8 7s-.8 4.8-2.8 7c-2-2.2-2.8-4.5-2.8-7S8 5.2 10 3z"/></svg>`,
    title: "The target and the environment",
  },
];

const UI_PANELS = {
  editor: UI.tabs({
    root: document.getElementById('editorTabs'), label: 'Left panel',
    items: EDITOR_TABS, initial: 'air',
    onChange(key) {
      $('.work').classList.toggle('wide', ['form', 'gs', 'ai'].includes(key));
      if (key === 'gs') renderGs(true);
      try { localStorage.setItem(LS + '-tab', key); } catch (e) {}
    },
  }),
  readouts: UI.tabs({
    root: document.getElementById('readoutTabs'), label: 'Readouts',
    items: READOUT_TABS, initial: 'flight', storageKey: 'drone-force-bench-v1-rtab',
    onChange(key) {
      if (key === 'flight') requestAnimationFrame(drawChart);
      if (key === 'health') renderHealth(true);
    },
  }),
};
