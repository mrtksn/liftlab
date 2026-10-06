# Chat Markdown dependencies

Pinned browser distributions copied from npm packages, served locally with the page:

- [Marked 18.1.0](https://github.com/markedjs/marked/releases/tag/v18.1.0), `lib/marked.umd.js`; MIT license in `marked-LICENSE`.
- [DOMPurify 3.4.16](https://github.com/cure53/DOMPurify/releases/tag/3.4.16), `dist/purify.min.js`; upstream licenses in `DOMPurify-LICENSE` and `DOMPurify-LICENSE-MPL`.

Downloads used `npm pack --ignore-scripts`. No runtime CDN or package installation is required. When upgrading, retain licenses and rerun chat rendering/link/HTML safety browser checks and `tools/stamp_ui_assets.py`.

`js/chat-markdown.js` renders assistant replies as GitHub-flavored Markdown and caches parsed content per message. Raw HTML is displayed as text; links allow HTTP(S), relative URLs and email. Images become links instead of loading remote resources. Tool output and user messages keep their plain-text formatting. Code blocks are displayed, never executed.
