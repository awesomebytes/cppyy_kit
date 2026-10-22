// Generator for roscon_uk_2026_deep_dive.pptx
// Run: NODE_PATH=<dir with node_modules> node build_deck.js <path to pptx skill dir> [out.pptx]
// Needs npm packages: pptxgenjs sharp react react-dom react-icons
const pptxgen = require("pptxgenjs");
const sharp = require("sharp");
const React = require("react");
const RDS = require("react-dom/server");
const fa = require("react-icons/fa");
const si = require("react-icons/si");
const path = require("path");
const SKILL = process.argv[2];
const OUT = process.argv[3] || "roscon_uk_2026_deep_dive.pptx";
const LOGO = path.resolve(__dirname, "../../docs/media/cppyy_kit_logo.jpg");
const { applyTheme } = require(path.join(SKILL, "scripts/apply_theme.js"));

const NAVY = "1E3A6E", YEL = "F5C518", BLUE = "3776AB", GREY = "F3F5F8", DARK = "1B2433",
  MUTED = "5B6578", BORDER = "D0D7DE", GREEN = "1F8A4C", RED = "C0392B", MID = "B8C2D3", WHITE = "FFFFFF";
const F = "Arial", M = "Courier New";
const ICONS = { python: fa.FaPython, db: fa.FaDatabase, lib: fa.FaCubes, ros: si.SiRos, ctrl: fa.FaSlidersH, cam: fa.FaVideo,
  robot: fa.FaRobot, file: fa.FaFileCode, clock: fa.FaHourglassHalf, play: fa.FaPlay, check: fa.FaCheckCircle, cross: fa.FaTimesCircle,
  bolt: fa.FaBolt, term: fa.FaTerminal, git: fa.FaGithub, book: fa.FaBook, chip: fa.FaMicrochip, hand: fa.FaHandPaper, quote: fa.FaQuoteLeft };
const COLS = { navy: NAVY, white: WHITE, yel: YEL, blue: BLUE, muted: "5B6B8C", green: GREEN, red: RED };
const IC = {};
async function prep() {
  for (const [n, C] of Object.entries(ICONS)) for (const [cn, hex] of Object.entries(COLS)) {
    const svg = RDS.renderToStaticMarkup(React.createElement(C, { color: "#" + hex, size: "256" }));
    IC[n + cn] = "image/png;base64," + (await sharp(Buffer.from(svg)).resize(256, 256, { fit: "contain", background: { r: 255, g: 255, b: 255, alpha: 0 } }).png().toBuffer()).toString("base64");
  }
}
const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE";
pres.theme = { headFontFace: F, bodyFontFace: F };
const THEME = { name: "cppyy_kit", headFontFace: F, bodyFontFace: F, colors: { dk1: DARK, lt1: WHITE, dk2: NAVY, lt2: GREY, accent1: NAVY, accent2: YEL, accent3: BLUE, accent4: GREEN, accent5: RED, accent6: MUTED, hlink: BLUE, folHlink: MUTED } };
const ph = (o) => ({ placeholder: { options: Object.assign({ name: "title", type: "title", fontFace: F, bold: true, color: NAVY, margin: 0, align: "left" }, o), text: "" } });
pres.defineSlideMaster({ title: "TITLE", background: { color: WHITE }, objects: [ph({ x: 0.8, y: 1.5, w: 6.3, h: 3.0, fontSize: 46, valign: "top" })] });
pres.defineSlideMaster({ title: "CONTENT", background: { color: WHITE }, objects: [ph({ x: 0.6, y: 0.3, w: 7.9, h: 1.1, fontSize: 32, valign: "middle" })],
  slideNumber: { x: 12.2, y: 7.02, w: 0.55, h: 0.3, fontFace: F, fontSize: 10, color: MUTED, align: "right" } });
pres.defineSlideMaster({ title: "DIVIDER", background: { color: WHITE }, objects: [ph({ x: 0.9, y: 1.75, w: 11.5, h: 1.7, fontSize: 44, valign: "top" })],
  slideNumber: { x: 12.2, y: 7.02, w: 0.55, h: 0.3, fontFace: F, fontSize: 10, color: MUTED, align: "right" } });

// ---------- helpers ----------
const SH = pres.shapes;
function T(s, text, x, y, w, h, o = {}) {
  s.addText(text, Object.assign({ isTextBox: true, x, y, w, h, fontFace: F, fontSize: 16, color: DARK, margin: 0, valign: "top" }, o));
}
function box(s, x, y, w, h, text, o = {}) {
  s.addText(text === undefined ? "" : text, Object.assign({ shape: o.round === false ? SH.RECTANGLE : SH.ROUNDED_RECTANGLE, rectRadius: o.round === false ? undefined : 0.1,
    x, y, w, h, fontFace: F, fontSize: 16, color: DARK, margin: 0.1, align: "left", valign: "middle",
    fill: { color: o.fill || GREY }, line: o.line ? { color: o.line, width: o.lw || 1 } : { type: "none" } }, o.t || {}));
}
function arrow(s, x1, y1, x2, y2, color = NAVY, w = 2, dash) {
  s.addShape(SH.LINE, { x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.abs(x2 - x1), h: Math.abs(y2 - y1), flipH: x2 < x1, flipV: y2 < y1,
    line: { color, width: w, endArrowType: "triangle", dashType: dash } });
}
function line(s, x1, y1, x2, y2, color, w = 1.5, dash) {
  s.addShape(SH.LINE, { x: Math.min(x1, x2), y: Math.min(y1, y2), w: Math.abs(x2 - x1), h: Math.abs(y2 - y1), flipV: y2 < y1, line: { color, width: w, dashType: dash } });
}
function ico(s, n, c, x, y, sz) { s.addImage({ data: IC[n + c], x, y, w: sz, h: sz }); }
function badgeNum(s, n, x, y) { box(s, x, y, 0.32, 0.32, String(n), { fill: YEL, t: { shape: SH.OVAL, align: "center", valign: "middle", bold: true, fontSize: 13, color: NAVY, margin: 0 } }); }
function stat(s, x, y, w, h, big, label, o = {}) {
  box(s, x, y, w, h, undefined, { fill: o.fill || GREY });
  T(s, big, x + 0.2, y + 0.15, w - 0.4, h * 0.5, { fontSize: o.fs || 44, bold: true, color: o.color || NAVY, valign: "middle" });
  T(s, label, x + 0.2, y + 0.15 + h * 0.5, w - 0.4, h * 0.5 - 0.25, { fontSize: 14, color: o.lcolor || DARK });
}
// stops
const STOPS = [["python", "Python"], ["db", "MCAP"], ["lib", "library"], ["ros", "ROS"], ["ctrl", "controller"], ["cam", "webcam"]];
function track(s, active, x0 = 8.65, y0 = 0.4, d = 0.46, pitch = 0.78) {
  for (let i = 0; i < 5; i++) line(s, x0 + d / 2 + i * pitch + d / 2, y0 + d / 2, x0 + d / 2 + (i + 1) * pitch - d / 2, y0 + d / 2, i < active ? NAVY : MID, 2);
  STOPS.forEach(([ic, lab], i) => {
    const st = i < active ? "past" : i === active ? "now" : "future";
    const cx = x0 + i * pitch;
    s.addShape(SH.OVAL, { x: cx, y: y0, w: d, h: d, fill: { color: st === "now" ? YEL : st === "past" ? NAVY : "E3E8F0" }, line: { type: "none" } });
    ico(s, ic, st === "now" ? "navy" : st === "past" ? "white" : "muted", cx + d * 0.25, y0 + d * 0.25, d * 0.5);
    T(s, lab, cx - 0.16, y0 + d + 0.05, d + 0.32, 0.2, { fontSize: 10, align: "center", color: st === "now" ? NAVY : MUTED, bold: st === "now" });
  });
}
function bigTrack(s, active) {
  const d = 1.0, pitch = 2.05, x0 = (13.333 - (5 * pitch + d)) / 2, y0 = 4.7;
  for (let i = 0; i < 5; i++) line(s, x0 + d + i * pitch, y0 + d / 2, x0 + (i + 1) * pitch, y0 + d / 2, i < active ? NAVY : MID, 3);
  STOPS.forEach(([ic, lab], i) => {
    const st = i < active ? "past" : i === active ? "now" : "future", cx = x0 + i * pitch;
    s.addShape(SH.OVAL, { x: cx, y: y0, w: d, h: d, fill: { color: st === "now" ? YEL : st === "past" ? NAVY : "E3E8F0" }, line: st === "now" ? { color: NAVY, width: 3 } : { type: "none" } });
    ico(s, ic, st === "now" ? "navy" : st === "past" ? "white" : "muted", cx + 0.25, y0 + 0.25, 0.5);
    T(s, lab, cx - 0.5, y0 + d + 0.15, d + 1, 0.35, { fontSize: 18, align: "center", color: st === "now" ? NAVY : MUTED, bold: st === "now" });
  });
}
// syntax colouring
const PYK = new Set("import from def return if for in not and or with as assert class while else elif is None True False".split(" "));
const CPK = new Set("const int static_cast unsigned long double for return size_t std".split(" "));
function lex(code, lang) {
  const kw = lang === "py" ? PYK : CPK, re = /(#.*|\/\/.*)|("[^"]*"|'[^']*')|(@\w+)|(\b\d[\d_]*\.?\d*(?:e\d+)?\b|\.\d+)|([A-Za-z_]\w*)|(\s+|.)/g;
  return code.split("\n").map((ln) => {
    const out = []; let m, prev = "";
    re.lastIndex = 0;
    while ((m = re.exec(ln))) {
      let c = "24292E";
      if (m[1]) c = "6A737D"; else if (m[2]) c = "032F62"; else if (m[3]) c = "6F42C1"; else if (m[4]) c = "005CC5";
      else if (m[5]) { if (kw.has(m[5])) c = "D73A49"; else if (prev === "def") c = "6F42C1"; else if (["print", "np", "cpp"].includes(m[5]) && lang === "py") c = "005CC5"; }
      if (m[5]) prev = m[5]; else if (m[6] && m[6].trim()) prev = "";
      out.push({ t: m[0], c });
    }
    return out;
  });
}
const CW = (fs) => 0.6 * fs / 72;
function code(s, name, src, lang, x, y, w, fs = 13, badges = []) {
  const lines = lex(src, lang), lh = fs * 1.3 / 72, h = 0.32 + lines.length * lh + 0.3;
  const longest = Math.max(...src.split("\n").map((l) => l.length));
  const gut = badges.length ? 0.45 : 0;
  if (longest * CW(fs) + 0.4 + gut > w) console.warn("CODE TOO WIDE", name, longest * CW(fs) + 0.4 + gut, w);
  box(s, x, y + 0.32, w, h - 0.32, undefined, { fill: "F6F8FA", line: BORDER, lw: 1 });
  box(s, x + 0.2, y, name.length * CW(12) + 0.4, 0.34, name, { fill: "E6EAF0", line: BORDER, t: { fontFace: M, fontSize: 12, color: DARK, align: "center", margin: 0 } });
  const runs = [];
  lines.forEach((ln, i) => {
    if (!ln.length) runs.push({ text: " ", options: { fontFace: M, fontSize: fs, breakLine: i < lines.length - 1, lineSpacing: fs * 1.3 } });
    ln.forEach((tk, j) => runs.push({ text: tk.t, options: { fontFace: M, fontSize: fs, color: tk.c, lineSpacing: fs * 1.3, breakLine: j === ln.length - 1 && i < lines.length - 1 } }));
  });
  T(s, runs, x + 0.2 + gut, y + 0.32 + 0.15, w - 0.4 - gut, lines.length * lh + 0.1);
  badges.forEach(([ln, n]) => badgeNum(s, n, x + 0.15, y + 0.32 + 0.15 + ln * lh + (lh - 0.32) / 2));
  return h;
}
const TC = { p: "50FA7B", c: "F8F8F2", cm: "9AA3B8", o: "C9D1E0", e: "F1FA8C" };
function term(s, title, lines, x, y, w, fs = 12) {
  const lh = fs * 1.4 / 72, inner = w - 0.6, cpl = Math.floor(inner / CW(fs));
  let n = 0; lines.forEach((l) => { const len = l.reduce((a, r) => a + r[0].length, 0); n += Math.max(1, Math.ceil(len / cpl)); });
  const h = 0.55 + n * lh + 0.3;
  box(s, x, y, w, h, undefined, { fill: "1E1E2E" });
  box(s, x, y, w, 0.42, undefined, { fill: "2B2D42" });
  s.addShape(SH.RECTANGLE, { x: x + 0.01, y: y + 0.25, w: w - 0.02, h: 0.17, fill: { color: "2B2D42" }, line: { type: "none" } });
  ["FF5F56", "FFBD2E", "27C93F"].forEach((c, i) => s.addShape(SH.OVAL, { x: x + 0.2 + i * 0.22, y: y + 0.14, w: 0.14, h: 0.14, fill: { color: c }, line: { type: "none" } }));
  T(s, title, x + 1, y + 0.1, w - 2, 0.25, { fontFace: M, fontSize: 11, color: "9AA3B8", align: "center" });
  const runs = [];
  lines.forEach((l, i) => l.forEach((r, j) => runs.push({ text: r[0], options: { fontFace: M, fontSize: fs, color: TC[r[1]], lineSpacing: fs * 1.4, breakLine: j === l.length - 1 && i < lines.length - 1 } })));
  T(s, runs, x + 0.3, y + 0.55, inner, n * lh + 0.1);
  return h;
}
const cmd = (c, env) => env ? [["$ ", "p"], [env + " ", "e"], [c, "c"]] : [["$ ", "p"], [c, "c"]];
function bars(s, x, y, w, rows, max, labelW = 2.7, rowH = 0.7) {
  const bw = w - labelW - 1.6;
  rows.forEach((r, i) => {
    const yy = y + i * rowH;
    T(s, r.label, x, yy, labelW - 0.15, rowH - 0.1, { fontSize: 14, valign: "middle", align: "right", bold: !!r.bold });
    s.addShape(SH.RECTANGLE, { x: x + labelW, y: yy + 0.05, w: Math.max(0.05, bw * r.v / max), h: rowH - 0.2, fill: { color: r.color }, line: { type: "none" } });
    T(s, r.text, x + labelW + Math.max(0.05, bw * r.v / max) + 0.12, yy, 1.5, rowH - 0.1, { fontSize: 16, bold: true, color: NAVY, valign: "middle" });
  });
}
let N = {};
function slide(layout, section, title, notes) {
  const s = pres.addSlide({ masterName: layout, sectionTitle: section });
  if (title) s.addText(title, { placeholder: "title" });
  if (notes) s.addNotes(notes);
  return s;
}
const content = (sec, title, active, notes) => { const s = slide("CONTENT", sec, title, notes); if (active !== null) track(s, active); return s; };
function divider(sec, idx, title, kicker, teaser, notes) {
  const s = slide("DIVIDER", sec, title, notes);
  T(s, kicker, 0.9, 1.2, 11, 0.4, { fontSize: 18, bold: true, color: BLUE });
  T(s, teaser, 0.9, 3.55, 11.5, 0.8, { fontSize: 18, color: MUTED });
  bigTrack(s, idx);
  return s;
}
const chipRow = (s, items, x, y, w, h, gap, o = {}) => { const cw = (w - gap * (items.length - 1)) / items.length; items.forEach((t, i) => box(s, x + i * (cw + gap), y, cw, h, t, Object.assign({ fill: GREY, line: BORDER, t: { align: "center", fontSize: 15, bold: true, color: NAVY } }, o))); };

// ---------- build ----------
(async () => {
  await prep();
  // 1 Title
  pres.addSection({ title: "Opening" });
  let s = slide("TITLE", "Opening", "It was supposed to be a small Python script", "A working spoken script for a roughly 45-minute deep dive. The story follows one prototype as we keep asking it to do more.");
  T(s, "A cppyy_kit deep dive", 0.8, 4.55, 6.3, 0.5, { fontSize: 26, color: BLUE, bold: true });
  box(s, 0.8, 5.3, 2.6, 0.55, "ROSCon UK 2026", { fill: YEL, t: { align: "center", bold: true, fontSize: 18, color: NAVY } });
  s.addImage({ path: LOGO, x: 7.6, y: 1.05, w: 5.3, h: 5.3 });
  track(s, -1, 0.8, 6.3);

  // 2 Opening question
  s = content("Opening", "Where does the interesting bit start?", -1, "So, say you're putting together a robot dataset. You've collected some recordings. You want to train a policy. Before that, you have a very reasonable question: where does the interesting bit actually start? The robot waits. Someone gets ready. Then it reaches for something. Then there's another pause. You'd quite like to cut some of that out before you spend your afternoon training a robot to wait patiently.");
  const seg = [["wait", 2.3, 0], ["get ready", 2.2, 0], ["reach", 2.6, 1], ["pause", 2.0, 0], ["reach", 2.6, 1]];
  let xx = 0.6;
  seg.forEach(([l, w, mv]) => { box(s, xx, 2.3, w, 1.1, l, { fill: mv ? YEL : "E3E8F0", t: { align: "center", bold: true, fontSize: 20, color: mv ? NAVY : "6B778C" } }); xx += w + 0.1; });
  ico(s, "clock", "muted", 1.55, 1.8, 0.4); ico(s, "hand", "navy", 7.3, 1.75, 0.45); ico(s, "hand", "navy", 11.75, 1.75, 0.45);
  arrow(s, 0.6, 3.75, 12.7, 3.75, MUTED, 1.5); T(s, "recording time", 10.7, 3.8, 2, 0.3, { fontSize: 12, color: MUTED, align: "right" });
  box(s, 0.6, 4.4, 5.6, 2.3, undefined, { fill: GREY });
  ico(s, "quote", "blue", 0.85, 4.65, 0.4);
  T(s, "Cut some of that out before you spend your afternoon training a robot to wait patiently.", 1.45, 4.6, 4.5, 1.6, { fontSize: 20, italic: true, color: NAVY, bold: true });
  ["Tell me when the hand starts moving", "Keep it active while it is moving", "Do not get excited about every tiny twitch"].forEach((t, i) => {
    box(s, 6.6, 4.4 + i * 0.8, 6.1, 0.7, t, { fill: WHITE, line: NAVY, lw: 1.5, t: { fontSize: 17, bold: true, color: NAVY, margin: 0.2 } });
  });

  // 3 Heuristic
  s = content("Opening", "A small motion rule", -1, "You don't need to solve all of robot learning today. You just want a little heuristic: tell me when the hand starts moving, keep it active while it's moving, and don't get excited about every tiny twitch. We have one speed to start moving and another to stop. We also want the movement to last a little while before we believe it. Otherwise we've built a very efficient noise detector.");
  const sp = []; for (let i = 0; i < 48; i++) sp.push(0.08 + 0.05 * ((i * 7) % 3));
  sp[8] = 0.62; [[10, 0.25], [11, 0.65], [12, 0.95], [13, 1.05], [14, 1.15], [15, 1.1], [16, 1.0], [17, 1.1], [18, 1.05], [19, 0.95], [20, 0.9], [21, 0.8], [22, 0.6], [23, 0.4], [24, 0.2]].forEach(([i, v]) => sp[i] = v);
  [[31, 0.3], [32, 0.7], [33, 1.0], [34, 1.1], [35, 1.2], [36, 1.05], [37, 1.0], [38, 1.1], [39, 0.9], [40, 0.7], [41, 0.45], [42, 0.25]].forEach(([i, v]) => sp[i] = v);
  const ENT = 0.55, EXT = 0.35, HOLD = 3, cx0 = 0.9, cy1 = 5.0, ch = 2.6, bw = 0.145;
  const mask = new Array(48).fill(0); let act = false;
  for (let i = 0; i < 48; i++) { if (!act) { let ok = true; for (let k = 0; k < HOLD; k++) if (!(i + k < 48 && sp[i + k] >= ENT)) ok = false; if (ok) act = true; } else if (sp[i] < EXT) act = false; mask[i] = act ? 1 : 0; }
  box(s, 0.6, 1.6, 8.2, 5.2, undefined, { fill: WHITE, line: BORDER });
  line(s, cx0, 1.9, cx0, cy1, MUTED, 1.5); line(s, cx0, cy1, 8.5, cy1, MUTED, 1.5);
  T(s, "speed", 0.7, 1.7, 0.9, 0.25, { fontSize: 12, color: MUTED });
  sp.forEach((v, i) => s.addShape(SH.RECTANGLE, { x: cx0 + 0.1 + i * 0.158, y: cy1 - v / 1.3 * ch, w: bw, h: v / 1.3 * ch, fill: { color: mask[i] ? NAVY : MID }, line: { type: "none" } }));
  line(s, cx0, cy1 - ENT / 1.3 * ch, 8.5, cy1 - ENT / 1.3 * ch, RED, 1.75, "dash"); line(s, cx0, cy1 - EXT / 1.3 * ch, 8.5, cy1 - EXT / 1.3 * ch, BLUE, 1.75, "dash");
  T(s, "enter speed", 6.6, cy1 - ENT / 1.3 * ch - 0.3, 1.9, 0.25, { fontSize: 12, bold: true, color: RED, align: "right" });
  T(s, "exit speed", 6.6, cy1 - EXT / 1.3 * ch + 0.04, 1.9, 0.25, { fontSize: 12, bold: true, color: BLUE, align: "right" });
  T(s, "tiny twitch, ignored", 1.2, 2.35, 2.3, 0.3, { fontSize: 12, bold: true, color: RED });
  arrow(s, 2.4, 2.65, 2.3, cy1 - sp[8] / 1.3 * ch - 0.05, RED, 1.25);
  const hx = cx0 + 0.1 + 11 * 0.158;
  line(s, hx, 2.4, hx + 3 * 0.158, 2.4, NAVY, 2); line(s, hx, 2.3, hx, 2.5, NAVY, 2); line(s, hx + 3 * 0.158, 2.3, hx + 3 * 0.158, 2.5, NAVY, 2);
  T(s, "hold time", hx - 0.4, 2.05, 1.5, 0.25, { fontSize: 12, bold: true, color: NAVY });
  T(s, "mask", 0.7, 5.35, 0.7, 0.3, { fontSize: 12, color: MUTED, bold: true });
  mask.forEach((m, i) => s.addShape(SH.RECTANGLE, { x: cx0 + 0.1 + i * 0.158, y: 5.45, w: bw + 0.013, h: 0.45, fill: { color: m ? YEL : "E3E8F0" }, line: { type: "none" } }));
  T(s, "active = 1 while moving", 1.0, 6.05, 4, 0.3, { fontSize: 12, color: MUTED });
  [["Enter speed", "Start moving above this", RED], ["Exit speed", "Stop below this", BLUE], ["Hold time", "Movement must last before we believe it", NAVY]].forEach(([a, b, c], i) => {
    box(s, 9.2, 1.6 + i * 1.2, 3.5, 1.05, undefined, { fill: GREY });
    s.addShape(SH.OVAL, { x: 9.4, y: 1.6 + i * 1.2 + 0.2, w: 0.25, h: 0.25, fill: { color: c }, line: { type: "none" } });
    T(s, a, 9.8, 1.6 + i * 1.2 + 0.12, 2.8, 0.35, { fontSize: 17, bold: true, color: NAVY }); T(s, b, 9.8, 1.6 + i * 1.2 + 0.5, 2.8, 0.5, { fontSize: 14 });
  });
  box(s, 9.2, 5.3, 3.5, 1.5, "Without the hold time we have built a very efficient noise detector.", { fill: YEL, t: { fontSize: 17, bold: true, color: NAVY, margin: 0.2 } });

  // 4 Journey
  s = content("Opening", "Six stops for one small script", null, "That's where we'll start. A small computation that's taking longer than we'd like. Then we'll do the usual thing that happens to a small robotics script: ask it to read a bag, run it in ROS, and eventually wonder whether it could be a controller. We'll finish by pointing a camera at something and moving it around. Timing budget: Python 7, MCAP 8, native library 4, ROS 6, controller 7, webcam 5.");
  const hooks = ["This loop is annoying me", "Someone gave us a humanoid bag", "A library we have not wrapped", "Could we just put it in ROS?", "Someone asks for a controller", "Can we make it follow this?"], mins = [7, 8, 4, 6, 7, 5];
  STOPS.forEach(([ic, lab], i) => {
    const x = 0.6 + i * 2.05;
    if (i < 5) arrow(s, x + 1.0 + 0.08, 2.5, x + 2.05 - 0.08, 2.5, MID, 2.5);
    s.addShape(SH.OVAL, { x: x + 0.1, y: 2.0, w: 1.0, h: 1.0, fill: { color: NAVY }, line: { type: "none" } }); ico(s, ic, "white", x + 0.35, 2.25, 0.5);
    box(s, x, 3.4, 1.9, 2.6, undefined, { fill: GREY });
    T(s, lab, x + 0.12, 3.55, 1.7, 0.35, { fontSize: 18, bold: true, color: NAVY });
    T(s, hooks[i], x + 0.12, 4.0, 1.7, 1.4, { fontSize: 14 });
    T(s, mins[i] + " min", x + 0.12, 5.5, 1.7, 0.35, { fontSize: 14, bold: true, color: BLUE });
  });
  T(s, "Then the same tool, asked for a little more each time.", 0.6, 6.35, 12, 0.4, { fontSize: 18, italic: true, color: MUTED });

  // 5 Divider
  pres.addSection({ title: "1 Python" });
  divider("1 Python", 0, "1. This loop is annoying me", "Python, 7 minutes", "Timestamps go in, hand positions go in, and a mask comes out telling us which samples count as moving.", "Here's our little motion detector. Timestamps go in, hand positions go in, and a mask comes out telling us which samples count as moving.");

  // 6 Agent workflow
  s = content("1 Python", "Agent writes, tests, times", 0, "I've supplied the boring bits in task.py: the input checks, the example data, and the entry point. The agent writes the missing logic. At this point I haven't told it anything about cppyy. Run the baseline prompt. Show the resulting function and its test result.");
  box(s, 0.6, 1.7, 2.9, 1.6, undefined, { fill: GREY });
  ico(s, "file", "blue", 0.8, 1.85, 0.4); T(s, "skeleton task.py", 1.3, 1.9, 2.1, 0.35, { fontSize: 16, bold: true, color: NAVY });
  T(s, "checks, example data, entry point", 0.8, 2.35, 2.5, 0.5, { fontSize: 14 }); T(s, "+ prompt", 0.8, 2.85, 2.5, 0.3, { fontSize: 14, bold: true, color: BLUE });
  box(s, 4.3, 1.7, 2.2, 1.6, undefined, { fill: NAVY }); ico(s, "robot", "yel", 4.9, 1.85, 0.6); T(s, "agent", 4.3, 2.6, 2.2, 0.4, { fontSize: 20, bold: true, color: WHITE, align: "center" });
  box(s, 7.3, 1.7, 2.4, 1.6, undefined, { fill: GREY }); ico(s, "python", "blue", 7.5, 1.85, 0.4); T(s, "implementation", 8.0, 1.9, 1.7, 0.35, { fontSize: 15, bold: true, color: NAVY }); T(s, "the missing logic", 7.5, 2.5, 2.1, 0.6, { fontSize: 14 });
  box(s, 10.5, 1.7, 2.2, 1.6, undefined, { fill: GREY }); ico(s, "check", "green", 10.7, 1.85, 0.4); T(s, "tests + timing", 11.2, 1.9, 1.5, 0.35, { fontSize: 15, bold: true, color: NAVY }); T(s, "pass / fail and milliseconds", 10.7, 2.5, 1.9, 0.7, { fontSize: 14 });
  arrow(s, 3.55, 2.5, 4.25, 2.5); arrow(s, 6.55, 2.5, 7.25, 2.5); arrow(s, 9.75, 2.5, 10.45, 2.5);
  T(s, "No cppyy mentioned yet", 3.2, 3.4, 4, 0.3, { fontSize: 14, italic: true, color: MUTED });
  term(s, "agent runner", [[["# 1. baseline: plain Python", "cm"]], cmd("pixi run python scripts/run_agent_eval.py implement --run rehearsal_01_python"), [["# 2. fresh session, kernel guide supplied", "cm"]], cmd("pixi run python scripts/run_agent_eval.py accelerate --run rehearsal_01_native --baseline rehearsal_01_python")], 0.6, 4.05, 12.1);

  // 7 Baseline
  s = content("1 Python", "NumPy plus a sequential loop", 0, "The agent gave us a sensible Python implementation. It already uses NumPy for the displacement and speed calculations. There's still a sequential loop deciding when a movement starts and ends. For 250,000 samples, that took about 29 milliseconds. We have options. We could try Numba, use Cython, or build a C++ extension with pybind11. I'm choosing cppyy because I also want to see what happens when our next request involves a C++ library or a native ROS interface.");
  stat(s, 0.6, 1.7, 4.4, 2.5, "29 ms", "for 250,000 samples, NumPy plus a sequential loop");
  box(s, 5.4, 1.7, 3.4, 2.5, "NumPy\ndisplacement and speed", { fill: GREY, t: { fontSize: 17, bold: true, color: NAVY, align: "center", margin: 0.2 } });
  arrow(s, 8.85, 2.95, 9.35, 2.95);
  box(s, 9.4, 1.7, 3.3, 2.5, "Python loop\nwhen does movement start and end?", { fill: YEL, t: { fontSize: 17, bold: true, color: NAVY, align: "center", margin: 0.2 } });
  T(s, "Routes we could explore", 0.6, 4.5, 6, 0.35, { fontSize: 18, bold: true, color: NAVY });
  ["Numba", "Cython", "pybind11"].forEach((t, i) => box(s, 0.6 + i * 2.2, 4.95, 2.0, 0.7, t, { fill: GREY, line: BORDER, t: { align: "center", fontSize: 17, bold: true, color: MUTED } }));
  box(s, 7.2, 4.95, 2.4, 0.7, "cppyy", { fill: YEL, t: { align: "center", fontSize: 20, bold: true, color: NAVY } });
  T(s, "I want to see what happens when the next request involves a C++ library or a native ROS interface.", 0.6, 5.95, 12, 0.8, { fontSize: 18, color: DARK });

  // 8 Whole idea
  s = content("1 Python", "The whole idea", 0, "For the moment, here's the whole idea in a tiny example. That string is the C++ body. The decorator compiles it, and we call it from Python. ConstNDArray[np.float64] supplies a const double* and the total element count as data_size. It borrows the buffer for the call and caches the compiled kernel for reuse.");
  const sumsq = `import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import ConstNDArray

@cpp(nogil=True)
def sum_sq(data: ConstNDArray[np.float64]) -> float:
    "double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i]*data[i]; return s;"

assert sum_sq(np.array([1, 2, 3], dtype=np.float64)) == 14.0`;
  code(s, "kernel.py", sumsq, "py", 0.6, 1.65, 12.1, 13, [[4, 3], [5, 2], [6, 1], [8, 4]]);
  [["1", "The string is the C++ body"], ["2", "ConstNDArray[np.float64] gives a const double* and data_size"], ["3", "nogil=True on the decorator"], ["4", "Compiled once, cached for reuse. The buffer is borrowed for the call"]].forEach(([n, t], i) => {
    box(s, 0.6 + i * 3.05, 4.9, 2.9, 1.8, undefined, { fill: GREY }); badgeNum(s, n, 0.8 + i * 3.05, 5.05); T(s, t, 0.8 + i * 3.05, 5.5, 2.5, 1.1, { fontSize: 15 });
  });

  // 9 Buffer contract
  s = content("1 Python", "The buffer contract", 0, "The array must have the specified dtype, native byte order, alignment and C-contiguous layout. The decorator rejects a mismatch; any normalization belongs in the Python wrapper. It borrows the buffer for the call. A mutable output uses NDArray[T].");
  box(s, 0.6, 1.8, 2.4, 1.9, "NumPy array", { fill: GREY, t: { align: "center", fontSize: 20, bold: true, color: NAVY } });
  box(s, 3.7, 1.7, 3.6, 2.1, undefined, { fill: WHITE, line: NAVY, lw: 2 });
  T(s, "Checks", 3.9, 1.8, 3, 0.35, { fontSize: 18, bold: true, color: NAVY });
  ["dtype", "native byte order", "alignment", "C-contiguous"].forEach((t, i) => { ico(s, "check", "green", 3.95, 2.28 + i * 0.37, 0.26); T(s, t, 4.35, 2.26 + i * 0.37, 2.8, 0.3, { fontSize: 15 }); });
  box(s, 8.0, 1.8, 2.0, 1.9, "borrowed pointer", { fill: YEL, t: { align: "center", fontSize: 17, bold: true, color: NAVY } });
  box(s, 10.7, 1.8, 2.0, 1.9, "C++ kernel\nconst double*\ndata_size", { fill: NAVY, t: { align: "center", fontSize: 16, bold: true, color: WHITE } });
  arrow(s, 3.05, 2.75, 3.65, 2.75); arrow(s, 7.35, 2.75, 7.95, 2.75); arrow(s, 10.05, 2.75, 10.65, 2.75);
  arrow(s, 5.5, 3.85, 5.5, 4.6, RED); T(s, "mismatch", 5.65, 4.0, 2, 0.3, { fontSize: 14, bold: true, color: RED });
  box(s, 4.0, 4.65, 3.0, 0.9, "rejected", { fill: WHITE, line: RED, lw: 2, t: { align: "center", fontSize: 20, bold: true, color: RED } });
  arrow(s, 3.95, 5.1, 3.0, 5.1, MUTED); box(s, 0.6, 4.65, 2.35, 0.9, "Python wrapper normalises", { fill: GREY, t: { align: "center", fontSize: 15, bold: true, color: NAVY } });
  arrow(s, 1.8, 4.6, 1.8, 3.75, MUTED);
  T(s, "A mutable output uses NDArray[T].", 8.0, 4.8, 4.7, 0.5, { fontSize: 16, color: DARK });
  T(s, "The kernel borrows the buffer for the call.", 8.0, 5.3, 4.7, 0.5, { fontSize: 16, color: DARK });

  // 10 Result
  s = content("1 Python", "About twenty times faster", 0, "In the original evaluation, twelve supplied test cases passed and independent random checks matched. The warmed computation went from about 29.3 milliseconds to 1.48. About twenty times faster on this input. We did pay for compilation and startup. We'll show that separately.");
  code(s, "python_native.py", `mask = np.zeros(n, dtype=np.int32)
if n < 2:
    return mask

_motion_mask_kernel(t, xyz.reshape(-1), n, enter_speed, exit_speed,
                    hold_seconds, mask)
return mask`, "py", 0.6, 1.65, 7.8, 13);
  stat(s, 8.6, 1.7, 4.1, 1.45, "12 tests", "supplied tests passed", { fs: 34, color: GREEN });
  stat(s, 8.6, 3.3, 4.1, 1.45, "match", "independent random checks matched", { fs: 34, color: GREEN });
  box(s, 0.6, 5.0, 12.1, 1.8, undefined, { fill: GREY });
  bars(s, 0.7, 5.1, 11.9, [{ label: "NumPy + Python loop", v: 29.3, text: "29.3 ms", color: MID }, { label: "cppyy native call", v: 1.48, text: "1.48 ms  (~20x)", color: NAVY, bold: true }], 29.3, 2.4, 0.8);

  // 11 Divider
  pres.addSection({ title: "2 MCAP" });
  divider("2 MCAP", 1, "2. Someone gave us a humanoid bag", "MCAP, 8 minutes", "An open humanoid manipulation recording. Our question is still quite modest: when are the hands moving?", "I went looking for an open humanoid manipulation recording and found this G1 pillow episode. It comes from HIW-500, by BitRobot, Unitree, and Hugging Face. Our question is still quite modest: when are the hands moving? How much does that answer change if I adjust the speed threshold?");

  // 12 Dataset
  s = content("2 MCAP", "A real recording", 1, "First discovery: the hand poses we want are inside JSON, inside a ROS String. So the loader gives us arrays of recorded timestamps and left/right hand states. There are 2,735 observations. That subtraction happens while the timestamps are still integers. Then we convert to seconds. I'd quite like our motion detector to use the time the data was recorded.");
  box(s, 0.6, 1.65, 8.0, 0.8, undefined, { fill: GREY });
  T(s, [{ text: "HIW-500 G1 pillow episode", options: { bold: true, color: NAVY, breakLine: true } }, { text: "by BitRobot, Unitree and Hugging Face", options: {} }], 0.85, 1.72, 7.6, 0.7, { fontSize: 15, valign: "middle" });
  stat(s, 8.9, 1.65, 3.8, 0.8, "", "", { fill: YEL }); T(s, [{ text: "2,735 ", options: { bold: true, fontSize: 26, color: NAVY } }, { text: "observations", options: { fontSize: 16, color: NAVY } }], 9.1, 1.65, 3.5, 0.8, { valign: "middle" });
  ["MCAP", "ROS String", "JSON", "ee_state", "NumPy arrays"].forEach((t, i) => { box(s, 0.6 + i * 2.5, 2.8, 2.1, 0.9, t, { fill: i === 4 ? NAVY : WHITE, line: NAVY, lw: 1.5, t: { align: "center", fontSize: 17, bold: true, color: i === 4 ? WHITE : NAVY } }); if (i < 4) arrow(s, 2.75 + i * 2.5, 3.25, 3.05 + i * 2.5, 3.25); });
  code(s, "analyze.py", `payload = json.loads(decoded.data)
state = payload["ee_state"]
timestamps.append(message.log_time)
states.append(state)

timestamps = np.asarray(timestamps, dtype=np.int64)
t = (timestamps - timestamps[0]).astype(np.float64) / 1e9`, "py", 0.6, 4.1, 7.2, 13);
  box(s, 8.2, 4.42, 4.5, 2.3, undefined, { fill: GREY });
  T(s, "Subtract while the timestamps are still integers, then convert to seconds.", 8.4, 4.55, 4.1, 1.2, { fontSize: 17, bold: true, color: NAVY });
  T(s, "The detector uses the time the data was recorded.", 8.4, 5.8, 4.1, 0.8, { fontSize: 15 });

  // 13 Sweep
  s = content("2 MCAP", "41 thresholds, two hands", 1, "Now we ask the agent to take our rule and sweep 41 speed thresholds for both hands. And here's a useful surprise from actually trying it. With the original 120-millisecond confirmation time, we got zero events. At 20 milliseconds, the lowest speed threshold gave us 89 events for the left hand and 82 for the right.");
  code(s, "sweep.py", `import numpy as np
from solutions.mcap_native import load_poses, sweep
ns, t, states = load_poses("data/hiw_pillow_episode_0002.mcap")
thresholds = np.linspace(.02, .20, 41)
counts, active_seconds = sweep(t, states, thresholds, hold_seconds=.02)
print(counts[0])  # [89, 82] at 0.02 m/s`, "py", 0.6, 1.65, 8.3, 13);
  T(s, "41 thresholds x 2 hands", 9.3, 1.75, 3.5, 0.3, { fontSize: 15, bold: true, color: NAVY });
  ["left", "right"].forEach((h, r) => { T(s, h, 9.3, 2.2 + r * 0.45, 0.5, 0.3, { fontSize: 12, color: MUTED }); for (let i = 0; i < 41; i++) s.addShape(SH.RECTANGLE, { x: 9.85 + i * 0.07, y: 2.2 + r * 0.45, w: 0.06, h: 0.35, fill: { color: i === 0 ? YEL : r ? BLUE : NAVY }, line: { type: "none" } }); });
  T(s, "0.02 to 0.20 m/s", 9.85, 3.15, 3, 0.3, { fontSize: 12, color: MUTED });
  stat(s, 0.6, 4.55, 5.6, 2.2, "0 events", "with the original 120 ms hold time");
  arrow(s, 6.3, 5.65, 6.9, 5.65, NAVY, 3);
  stat(s, 7.0, 4.55, 5.7, 2.2, "89 left / 82 right", "with a 20 ms hold, lowest speed threshold", { fill: YEL, fs: 36 });

  // 14 171
  s = content("2 MCAP", "Have we discovered 171 successful grasps?", 1, "So, have we discovered 171 successful grasps? We have discovered 171 things our motion rule calls events. Let's look at some of them before we put those labels into a training dataset. Even the annotations gave us something to discuss: the episode name mentions a sofa and floor, while the subtask labels mention a bed and chair. Welcome to working with somebody else's data.");
  box(s, 0.6, 1.65, 6.9, 3.5, undefined, { fill: GREY });
  for (let r = 0; r < 9; r++) for (let c = 0; c < 19; c++) { const k = r * 19 + c; s.addShape(SH.OVAL, { x: 0.9 + c * 0.32, y: 1.95 + r * 0.32, w: 0.22, h: 0.22, fill: { color: k < 89 ? NAVY : BLUE }, line: { type: "none" } }); }
  T(s, "89 left hand + 82 right hand = 171 events", 0.9, 4.85, 6.4, 0.3, { fontSize: 14, bold: true, color: NAVY });
  box(s, 0.6, 5.4, 3.4, 1.4, "Have we discovered 171 successful grasps?", { fill: WHITE, line: NAVY, lw: 1.5, t: { fontSize: 16, bold: true, color: NAVY, margin: 0.15 } });
  box(s, 4.1, 5.4, 3.4, 1.4, "We have discovered 171 things our motion rule calls events.", { fill: YEL, t: { fontSize: 16, bold: true, color: NAVY, margin: 0.15 } });
  box(s, 7.9, 1.65, 4.8, 5.15, undefined, { fill: WHITE, line: BORDER });
  T(s, "Even the labels disagree", 8.15, 1.8, 4.3, 0.4, { fontSize: 19, bold: true, color: NAVY });
  T(s, "Episode name", 8.15, 2.4, 4.3, 0.3, { fontSize: 14, color: MUTED }); chipRow(s, ["sofa", "floor"], 8.15, 2.75, 4.3, 0.6, 0.2);
  T(s, "not equal", 8.15, 3.5, 4.3, 0.4, { fontSize: 20, bold: true, color: RED, align: "center" });
  T(s, "Subtask labels", 8.15, 4.0, 4.3, 0.3, { fontSize: 14, color: MUTED }); chipRow(s, ["bed", "chair"], 8.15, 4.35, 4.3, 0.6, 0.2);
  T(s, "Welcome to working with somebody else's data. We keep the source labels visible and inspect the recording.", 8.15, 5.2, 4.3, 1.4, { fontSize: 15 });

  // 15 Timing
  s = content("2 MCAP", "Query is fast, decoding dominates", 1, "The recorded query went from about 24.8 milliseconds in Python to 1.14 in C++. About twenty-two times faster. Loading and decoding the bag still took about 191 milliseconds in that run. So if I reload the whole file every time, that part still dominates. If I've already got the arrays and I'm experimenting with the rule, the faster query is useful straight away.");
  box(s, 0.6, 1.7, 12.1, 2.9, undefined, { fill: GREY });
  bars(s, 0.7, 1.9, 11.9, [{ label: "Load + decode bag", v: 191, text: "191 ms", color: MID }, { label: "Query in Python", v: 24.8, text: "24.8 ms", color: BLUE }, { label: "Query in C++", v: 1.14, text: "1.14 ms  (~22x)", color: NAVY, bold: true }], 191, 2.4, 0.85);
  box(s, 0.6, 4.9, 5.9, 1.8, "Reload the whole file every time: decoding still dominates.", { fill: WHITE, line: BORDER, t: { fontSize: 18, bold: true, color: NAVY, margin: 0.25 } });
  box(s, 6.8, 4.9, 5.9, 1.8, "Arrays already loaded, tuning the rule: the faster query helps straight away.", { fill: YEL, t: { fontSize: 18, bold: true, color: NAVY, margin: 0.25 } });

  // 16 Divider
  pres.addSection({ title: "3 Native library" });
  divider("3 Native library", 2, "3. Could we add a library we have not wrapped?", "Native library, 4 minutes", "Nearby observations with enough confidence and enough separation in recorded frames, plus their centroid.", "Suppose our next experiment needs nearby Cartesian observations, but only those with sufficient confidence and enough separation in recorded frames. We also want the centroid of the selected observations.");

  // 17 Architecture
  s = content("3 Native library", "A library without a new kit", 2, "The nanoflann recipe does that without creating a nanoflann kit. It pins and checks the header, builds a small native adapter, loads its declarations, and retains the index between queries. Python validates arrays and allocates outputs. C++ owns a copy of indexed storage, traverses the actual nanoflann tree, filters candidates and writes neighbors, squared distances, counts and centroids into output arrays.");
  box(s, 0.6, 1.65, 3.3, 2.2, undefined, { fill: GREY }); T(s, "Python side", 0.8, 1.75, 3, 0.35, { fontSize: 18, bold: true, color: BLUE });
  T(s, "validate arrays\nallocate outputs\nrun the experiment", 0.8, 2.2, 3, 1.4, { fontSize: 16 });
  box(s, 4.5, 1.65, 4.6, 2.2, undefined, { fill: WHITE, line: NAVY, lw: 2 }); T(s, "Native adapter", 4.7, 1.75, 4.2, 0.35, { fontSize: 18, bold: true, color: NAVY });
  T(s, "pinned nanoflann header\nretained index, owned copy of points\ntraversal + filter + centroid", 4.7, 2.2, 4.3, 1.5, { fontSize: 15 });
  box(s, 9.7, 1.65, 3.0, 2.2, undefined, { fill: GREY }); T(s, "Output arrays", 9.9, 1.75, 2.7, 0.35, { fontSize: 18, bold: true, color: BLUE });
  T(s, "neighbors\nsquared distances\ncounts, centroids", 9.9, 2.2, 2.7, 1.4, { fontSize: 15 });
  arrow(s, 3.95, 2.75, 4.45, 2.75); arrow(s, 9.15, 2.75, 9.65, 2.75);
  ["Pin and check the header", "Build a small native adapter", "Load its declarations", "Retain the index between queries"].forEach((t, i) => { box(s, 0.6 + i * 3.05, 4.1, 2.9, 0.75, undefined, { fill: GREY }); badgeNum(s, i + 1, 0.72 + i * 3.05, 4.32); T(s, t, 1.12 + i * 3.05, 4.1, 2.3, 0.75, { fontSize: 14, valign: "middle" }); });
  code(s, "demo.py", `from index import Index

with Index(points, frame_ids, confidence) as index:
    result = index.query(query_points, query_frames, k=8, frame_gap=128,
                         min_confidence=.6, max_distance=.25)
    print(result["centroids"])`, "py", 0.6, 5.0, 8.4, 12);
  T(s, "Python controls the experiment. C++ keeps the index and does the traversal.", 9.3, 5.4, 3.4, 1.2, { fontSize: 15, color: NAVY, bold: true });

  // 18 Results
  s = content("3 Native library", "Custom rule plus traversal", 2, "The 4 October experiment used 50,000 synthetic positions and 2,000 queries. The warmed composed native call took 3.19 milliseconds. The adaptive SciPy search plus Python selection and centroid calculation took 31.49 milliseconds. Direct unfiltered searches were close: 1.95 milliseconds native, including the centroid, and 2.32 for SciPy without it. A complete agent run took 513 seconds.");
  box(s, 0.6, 1.7, 8.2, 5.0, undefined, { fill: GREY });
  T(s, "Composed query with selection rule", 0.8, 1.85, 7, 0.35, { fontSize: 16, bold: true, color: NAVY });
  bars(s, 0.7, 2.3, 8.0, [{ label: "SciPy + Python selection", v: 31.49, text: "31.49 ms", color: MID }, { label: "Native composed call", v: 3.19, text: "3.19 ms", color: NAVY, bold: true }], 31.49, 2.3, 0.8);
  T(s, "Direct searches", 0.8, 4.3, 7, 0.35, { fontSize: 16, bold: true, color: NAVY });
  bars(s, 0.7, 4.75, 8.0, [{ label: "SciPy", v: 2.32, text: "2.32 ms", color: MID }, { label: "Native, incl. centroid", v: 1.95, text: "1.95 ms", color: NAVY, bold: true }], 31.49, 2.3, 0.8);
  stat(s, 9.1, 1.7, 3.6, 1.5, "50,000", "synthetic positions, 2,000 queries", { fs: 32 });
  stat(s, 9.1, 3.4, 3.6, 1.5, "3.19 ms", "warmed composed native call", { fs: 32, fill: YEL });
  stat(s, 9.1, 5.1, 3.6, 1.6, "513 s", "one complete agent run", { fs: 32 });

  // 19 Divider
  pres.addSection({ title: "4 ROS" });
  divider("4 ROS", 3, "4. Could we just put it in ROS?", "ROS, 6 minutes", "A subscriber that keeps the latest 256 observations and runs the same query on that window.", "Of course we can put it in ROS. There's a subscriber already written in node.py. It decodes the message, keeps the latest 256 observations, and runs the same query on that window.");

  // 20 Node
  s = content("4 ROS", "The same query in a ROS node", 3, "Now our computation has a regular job: finish its work before the next round of data gives it more to do. We're playing the recording at four times its original speed. The replay adapter puts the original timestamps and a sequence number into each standard ROS message. Our speed calculation still uses recorded time. Playing a bag faster shouldn't persuade the heuristic that the robot grew faster arms.");
  box(s, 0.6, 1.7, 2.6, 1.6, undefined, { fill: GREY }); ico(s, "play", "blue", 0.8, 1.85, 0.4); T(s, "ros2 bag play", 1.3, 1.88, 1.9, 0.35, { fontSize: 15, bold: true, color: NAVY }); T(s, "at 4x speed", 0.8, 2.5, 2.2, 0.4, { fontSize: 14 });
  arrow(s, 3.25, 2.5, 4.05, 2.5); T(s, "/roscon/poses", 3.1, 2.0, 1.4, 0.3, { fontSize: 12, bold: true, color: BLUE });
  box(s, 4.1, 1.7, 6.2, 1.6, undefined, { fill: WHITE, line: NAVY, lw: 2 }); T(s, "callback", 4.3, 1.75, 3, 0.3, { fontSize: 14, bold: true, color: NAVY });
  ["JSON decode", "window of 256", "rolling_query (native)"].forEach((t, i) => { box(s, 4.3 + i * 1.95, 2.2, 1.8, 0.9, t, { fill: i === 2 ? YEL : GREY, t: { align: "center", fontSize: 13, bold: true, color: NAVY, margin: 0.05 } }); if (i < 2) arrow(s, 6.1 + i * 1.95, 2.65, 6.25 + i * 1.95, 2.65, NAVY, 1.5); });
  arrow(s, 10.35, 2.5, 11.0, 2.5); box(s, 11.05, 1.7, 1.65, 1.6, "counts per window", { fill: NAVY, t: { align: "center", fontSize: 15, bold: true, color: WHITE } });
  code(s, "node.py", `history.append((t, state))
times = np.ascontiguousarray([item[0] for item in history])
states = np.ascontiguousarray([item[1] for item in history])
counts, seconds = rolling_query(times, states, thresholds)`, "py", 0.6, 3.7, 7.5, 13);
  box(s, 8.5, 3.7, 4.2, 3.0, undefined, { fill: YEL });
  T(s, "Recorded time, not wall time", 8.7, 3.85, 3.8, 0.7, { fontSize: 19, bold: true, color: NAVY });
  T(s, "Playing a bag faster should not persuade the heuristic that the robot grew faster arms.", 8.7, 4.65, 3.8, 1.2, { fontSize: 15, color: NAVY });
  T(s, "Counts describe the current window.", 8.7, 5.95, 3.8, 0.6, { fontSize: 14, color: NAVY });

  // 21 Demo + results
  s = content("4 ROS", "Two terminals, same ROS domain", 3, "For a two-terminal demonstration, start the node first. Both terminals need the same ROS domain. Both original playback trials received all 2,735 messages, in order, and produced matching event counts. The median callback went from about 2.84 milliseconds to 0.50. There's still JSON decoding, checking, and assembling the window around the accelerated computation. That's why the whole callback gets a smaller speedup than the loop by itself.");
  const h1 = term(s, "terminal 1: the node", [cmd("pixi run -e ros python solutions/ros_native.py", "ROS_DOMAIN_ID=78")], 0.6, 1.65, 12.1);
  term(s, "terminal 2: the bag", [cmd("pixi run -e ros ros2 bag play -i data/poses_replay.mcap mcap --rate 4 --delay 1 --topics /roscon/poses --disable-keyboard-controls", "ROS_DOMAIN_ID=78")], 0.6, 1.65 + h1 + 0.2, 12.1);
  stat(s, 0.6, 4.75, 3.9, 2.0, "2,735 / 2,735", "messages received in order, matching event counts", { fs: 30, color: GREEN });
  box(s, 4.75, 4.75, 3.9, 2.0, undefined, { fill: GREY }); T(s, "Median callback", 4.95, 4.85, 3.5, 0.3, { fontSize: 14, bold: true });
  bars(s, 4.8, 5.2, 3.8, [{ label: "", v: 2.84, text: "2.84 ms", color: MID }, { label: "", v: 0.5, text: "0.50 ms", color: NAVY }], 2.84, 0.1, 0.6);
  box(s, 8.9, 4.75, 3.8, 2.0, "Smaller speedup than the bare loop: JSON decoding, checking and window assembly remain.", { fill: YEL, t: { fontSize: 15, bold: true, color: NAVY, margin: 0.2 } });

  // 22 Divider
  pres.addSection({ title: "5 Controller" });
  divider("5 Controller", 4, "5. At this point, someone asks for a controller", "Controller, 7 minutes", "Could we write a ros2_control controller in Python? Let's try it.", "We've got Python talking to native code. We have ROS running. So naturally someone asks: could we write a ros2_control controller in Python? Let's try it.");

  // 23 Controller diagram
  s = content("5 Controller", "A ros2_control controller in Python", 4, "The skeleton in controller.py derives from ck.ControllerInterface. That's the native controller interface, exposed through control_kit. The lifecycle methods and interface declarations are supplied. The agent fills in the update. This example runs inside a real controller_manager, with mock hardware. We read two joint positions, follow a moving reference, and write the commands back through the claimed interfaces.");
  box(s, 0.6, 1.65, 4.3, 0.9, "controller_manager", { fill: GREY, t: { align: "center", fontSize: 17, bold: true, color: NAVY } });
  arrow(s, 2.75, 2.6, 2.75, 3.05); T(s, "update(period)", 2.9, 2.65, 2, 0.3, { fontSize: 12, color: MUTED });
  box(s, 0.6, 3.1, 4.3, 1.1, undefined, { fill: NAVY }); ico(s, "python", "yel", 0.8, 3.35, 0.5);
  T(s, [{ text: "TrackingController", options: { bold: true, breakLine: true } }, { text: "Python, derives ck.ControllerInterface", options: { fontSize: 12 } }], 1.45, 3.15, 3.4, 1, { fontSize: 16, color: WHITE, valign: "middle" });
  arrow(s, 2.3, 4.25, 2.3, 4.7); arrow(s, 3.2, 4.7, 3.2, 4.25); T(s, "read", 1.75, 4.3, 0.5, 0.3, { fontSize: 12, color: MUTED }); T(s, "write", 3.3, 4.3, 0.6, 0.3, { fontSize: 12, color: MUTED });
  box(s, 0.6, 4.75, 4.3, 0.7, "claimed state / command interfaces", { fill: YEL, t: { align: "center", fontSize: 14, bold: true, color: NAVY } });
  arrow(s, 2.75, 5.5, 2.75, 5.85); box(s, 0.6, 5.9, 4.3, 0.85, "GenericSystem mock hardware", { fill: GREY, t: { align: "center", fontSize: 16, bold: true, color: NAVY } });
  code(s, "controller.py", `dt = period.seconds()
self.t += dt
references = [0.3 * math.sin(self.t), 0.2 * math.cos(self.t)]
positions = []
for i in range(ck.n_state_interfaces(self)):
    position = ck.read_state(self, i)
    command = position + dt * 20.0 * (references[i] - position)
    ck.write_command(self, i, command)
    positions.append(position)
self.samples.append((self.t, references, positions))
return ck.return_type.OK`, "py", 5.3, 1.65, 7.4, 12);
  box(s, 5.3, 5.5, 7.4, 1.25, "Lifecycle methods and interface declarations are supplied. The agent fills in update().", { fill: YEL, t: { fontSize: 16, bold: true, color: NAVY, margin: 0.2 } });

  // 24 Controller results
  s = content("5 Controller", "A thousand updates a second", 4, "In the original controller evaluation, we asked for a thousand updates a second. The agent's three-second run achieved about 996 Hz. The independent run, while other checks were active, was about 989 Hz. Its tracking error in the second half was about 0.016 radians. Now, before anyone makes this slide their robot's new control architecture: look at the timing distribution too. That independent run had a 3.89-millisecond p99 interval, and 203 intervals longer than 1.5 milliseconds.");
  [["996 Hz", "agent run, 3 seconds"], ["989 Hz", "independent run"], ["3.89 ms", "p99 interval"], ["203", "intervals above 1.5 ms"], ["0.016 rad", "tracking error, second half"]].forEach(([a, b], i) => stat(s, 0.6 + i * 2.45, 1.7, 2.25, 1.9, a, b, { fs: 30, fill: i === 2 || i === 3 ? YEL : GREY }));
  box(s, 0.6, 3.95, 12.1, 1.5, undefined, { fill: WHITE, line: BORDER });
  T(s, "Interval scale (ms)", 0.8, 4.05, 3, 0.3, { fontSize: 13, bold: true, color: MUTED });
  const ax = (v) => 1.0 + v / 4.2 * 11.2;
  line(s, 1.0, 5.05, 12.2, 5.05, MUTED, 2);
  [[0, "0"], [1, "1.0 target"], [1.5, "1.5"], [3.89, "3.89 p99"]].forEach(([v, l], i) => { s.addShape(SH.OVAL, { x: ax(v) - 0.09, y: 4.96, w: 0.18, h: 0.18, fill: { color: i === 3 ? RED : i === 2 ? BLUE : NAVY }, line: { type: "none" } }); T(s, l, ax(v) - 0.7, 5.2, 1.4, 0.25, { fontSize: 12, align: "center", bold: true, color: DARK }); });
  T(s, "late intervals above 1.5 ms", ax(1.5) + 0.2, 4.5, 4, 0.3, { fontSize: 12, color: BLUE, bold: true });
  box(s, 0.6, 5.75, 12.1, 1.05, "The average rate is only part of the story. Look at the timing distribution too.", { fill: YEL, t: { fontSize: 20, bold: true, color: NAVY, margin: 0.3 } });

  // 25 Divider
  pres.addSection({ title: "6 Webcam" });
  divider("6 Webcam", 5, "6. Can we make it follow this?", "Webcam, 5 minutes", "Click on a little patch and follow it as I move it around.", "Here's the webcam. I want to click on a little patch and follow it as I move it around. A small image-processing task where we can see what the code is doing.");

  // 26 Webcam algorithm
  s = content("6 Webcam", "Find the most similar patch", 5, "The missing bit searches for the most similar patch in the next frame: an 11-by-11 template, with a search of eight pixels in each direction. Inside the search, we're comparing pixel values and adding up squared differences. Convert the pixel values before subtracting them, and use a wide accumulator.");
  const cs = 0.2; T(s, "template: 11 x 11 px", 0.6, 1.6, 3, 0.3, { fontSize: 14, bold: true, color: NAVY });
  s.addShape(SH.RECTANGLE, { x: 0.6, y: 1.95, w: 11 * cs, h: 11 * cs, fill: { color: YEL }, line: { color: NAVY, width: 1.5 } });
  for (let i = 1; i < 11; i++) { line(s, 0.6 + i * cs, 1.95, 0.6 + i * cs, 1.95 + 11 * cs, NAVY, 0.5); line(s, 0.6, 1.95 + i * cs, 0.6 + 11 * cs, 1.95 + i * cs, NAVY, 0.5); }
  const g = 0.115, gx = 3.6, gy = 1.95; T(s, "search window: +/- 8 px", gx, 1.6, 3.5, 0.3, { fontSize: 14, bold: true, color: NAVY });
  s.addShape(SH.RECTANGLE, { x: gx, y: gy, w: 27 * g, h: 27 * g, fill: { color: GREY }, line: { color: NAVY, width: 1.5 } });
  for (let i = 1; i < 27; i++) { line(s, gx + i * g, gy, gx + i * g, gy + 27 * g, MID, 0.4); line(s, gx, gy + i * g, gx + 27 * g, gy + i * g, MID, 0.4); }
  s.addShape(SH.RECTANGLE, { x: gx + 8 * g, y: gy + 8 * g, w: 11 * g, h: 11 * g, fill: { color: YEL }, line: { color: NAVY, width: 2 } });
  s.addShape(SH.RECTANGLE, { x: gx + 13 * g, y: gy + 4 * g, w: 11 * g, h: 11 * g, fill: { type: "none" }, line: { color: RED, width: 2, dashType: "dash" } });
  T(s, "candidate position, scored by sum of squared differences", 0.6, 4.35, 2.8, 0.9, { fontSize: 12, color: MUTED });
  [["1", "Cast to int before subtracting"], ["2", "Use a wide accumulator"], ["3", "Exact integer scoring, row-major tie breaking, clipping"]].forEach(([n, t], i) => { box(s, 7.4, 1.7 + i * 0.95, 5.3, 0.8, undefined, { fill: GREY }); badgeNum(s, n, 7.55, 1.94 + i * 0.95); T(s, t, 8.05, 1.7 + i * 0.95, 4.5, 0.8, { fontSize: 15, valign: "middle" }); });
  code(s, "tracker.cpp", `const int difference = static_cast<int>(previous[previous_row + x + dx])
                     - static_cast<int>(current[current_row + cx + dx]);
score += static_cast<unsigned long long>(difference * difference);`, "cpp", 0.6, 5.4, 8.8, 13, [[0, 1], [2, 2]]);

  // 27 Webcam results
  s = content("6 Webcam", "From synthetic to live frames", 5, "The original synthetic search measured about 0.026 milliseconds once warm. Building its kernel took about 546 milliseconds. The actual headless webcam check tracked 30 frames at about 0.20 milliseconds per search. It's a small local tracker, so big jumps and occlusion can lose it, and it can drift. That's useful to show too: we can see where this particular algorithm stops helping and start changing it.");
  stat(s, 0.6, 1.7, 3.9, 1.9, "0.026 ms", "warm search, synthetic", { fs: 40 });
  stat(s, 4.75, 1.7, 3.9, 1.9, "546 ms", "kernel build", { fs: 40 });
  stat(s, 8.9, 1.7, 3.8, 1.9, "0.20 ms", "per search, 30 live frames", { fs: 40, fill: YEL });
  box(s, 0.6, 3.95, 5.9, 2.8, undefined, { fill: GREY }); T(s, "Where it stops helping", 0.85, 4.1, 5.4, 0.4, { fontSize: 19, bold: true, color: NAVY });
  ["Big jumps", "Occlusion", "Drift"].forEach((t, i) => { ico(s, "cross", "red", 0.9, 4.7 + i * 0.6, 0.35); T(s, t, 1.45, 4.68 + i * 0.6, 4.8, 0.4, { fontSize: 18 }); });
  box(s, 6.8, 3.95, 5.9, 2.8, undefined, { fill: NAVY }); ico(s, "cam", "yel", 7.05, 4.15, 0.6);
  T(s, "LIVE DEMO", 7.85, 4.2, 4.5, 0.5, { fontSize: 26, bold: true, color: YEL });
  T(s, "Click a textured patch and move it slowly. Then show what happens when it loses the patch.", 7.05, 5.0, 5.4, 1.5, { fontSize: 18, color: WHITE });

  // 28 Recap
  pres.addSection({ title: "Closing" });
  s = content("Closing", "A script with responsibilities", 6, "We asked a script to find some movement. Then it became a dataset query. We added a native library and a custom selection rule. Then we wanted the motion query in a ROS callback. Then we tried a controller. Finally, we gave it an image-processing job we could poke at live.");
  const chain = [["python", "loop"], ["db", "dataset tool"], ["lib", "+ C++ library"], ["ros", "ROS callback"], ["ctrl", "controller"], ["cam", "live tracker"]];
  chain.forEach(([ic, lab], i) => {
    const h = 1.5 + i * 0.6, x = 0.6 + i * 2.05, y = 6.7 - h;
    box(s, x, y, 1.9, h, undefined, { fill: i === 5 ? YEL : i % 2 ? GREY : "E3E8F0" });
    s.addShape(SH.OVAL, { x: x + 0.6, y: y + 0.2, w: 0.7, h: 0.7, fill: { color: NAVY }, line: { type: "none" } }); ico(s, ic, "white", x + 0.75, y + 0.35, 0.4);
    T(s, lab, x + 0.1, y + 1.0, 1.7, 0.4, { fontSize: 15, bold: true, color: NAVY, align: "center" });
    if (i < 5) arrow(s, x + 1.92, 6.0 - i * 0.3, x + 2.03, 6.0 - i * 0.3, MUTED, 2);
  });
  T(s, "Our small script acquired a few responsibilities.", 0.6, 1.7, 8, 0.4, { fontSize: 20, italic: true, color: MUTED });

  // 29 Why cppyy
  s = content("Closing", "Why cppyy?", 6, "That's what interests me about cppyy. If I only need to speed up a numerical loop, I've got several tools I can try. But I often want to keep adding things: a C++ library that's already doing useful work, a native ROS API, or a controller interface I'd like to experiment with from Python. cppyy_kit gives us the cached kernels and kit setup to work with those pieces. The agent gets explicit guidance, and we get to spend more of the session asking what the robot data means and what the program should do next.");
  box(s, 0.6, 1.7, 5.7, 3.6, undefined, { fill: GREY }); T(s, "Speed up one loop", 0.85, 1.85, 5.2, 0.4, { fontSize: 22, bold: true, color: MUTED });
  T(s, "Many tools", 0.85, 2.35, 5.2, 0.4, { fontSize: 18 }); chipRow(s, ["Numba", "Cython", "pybind11"], 0.85, 2.95, 5.2, 0.7, 0.15);
  box(s, 7.0, 1.7, 5.7, 3.6, undefined, { fill: YEL }); T(s, "Keep adding things", 7.25, 1.85, 5.2, 0.4, { fontSize: 22, bold: true, color: NAVY });
  T(s, "C++ libraries already doing useful work\nNative ROS APIs\nController interfaces to try from Python", 7.25, 2.5, 5.2, 1.6, { fontSize: 18, color: NAVY });
  T(s, "cppyy_kit: cached kernels, kit setup, explicit guidance for the agent", 7.25, 4.2, 5.2, 0.9, { fontSize: 16, bold: true, color: NAVY });
  arrow(s, 6.35, 3.5, 6.95, 3.5, NAVY, 3);
  box(s, 0.6, 5.65, 12.1, 1.1, "Successful agent runs took roughly 42 to 220 seconds across the five stages.", { fill: WHITE, line: NAVY, lw: 1.5, t: { fontSize: 18, bold: true, color: NAVY, margin: 0.3 } });

  // 30 Try it
  s = content("Closing", "Try it", 6, "For now, take a bit of Python you wish was quicker. Ask your agent to implement it, test it, and measure it. Then give it the cppyy_kit guide and try moving the busy part into one native call. See whether the result is useful for your problem. And if your next thought is 'could we also add...', that's the bit I'd like to explore with you.");
  term(s, "install and guides", [[["# install Pixi first, then:", "cm"]], cmd('pixi init cppyy-example -c https://prefix.dev/awesomebytes -c robostack-jazzy -c conda-forge'), cmd("cd cppyy-example"), cmd('pixi add "cppyy-kit>=0.4.1" ros-jazzy-bt-kit numpy'), [["# guides for your agent", "cm"]], cmd("pixi run python -m cppyy_kit guide accelerate"), cmd("pixi run python -m cppyy_kit guide bring-library"), cmd("pixi run python -m cppyy_kit guide existing-cpp")], 0.6, 1.6, 12.1);
  box(s, 0.6, 5.0, 12.1, 0.8, "Take a bit of Python you wish was quicker. Ask your agent to implement it, test it, measure it. Then move the busy part into one native call.", { fill: YEL, t: { fontSize: 16, bold: true, color: NAVY, margin: 0.25 } });
  [["git", "GitHub repo and README", "github.com/awesomebytes/cppyy_kit"], ["book", "Native-component tutorial", "docs/tutorials/native_component.md"], ["term", "Getting started", "awesomebytes.github.io/cppyy_kit/getting-started"]].forEach(([ic, a, b], i) => {
    box(s, 0.6 + i * 4.1, 6.0, 3.9, 0.85, undefined, { fill: GREY }); ico(s, ic, "navy", 0.75 + i * 4.1, 6.2, 0.4);
    T(s, [{ text: a, options: { bold: true, color: NAVY, breakLine: true } }, { text: b, options: { fontSize: 12, color: BLUE } }], 1.3 + i * 4.1, 6.05, 2.6, 0.75, { fontSize: 14, valign: "middle" });
  });

  // 31 Thanks
  s = slide("CONTENT", "Closing", "Thanks, questions?", "Questions. The dataset is the HIW-500 G1 pillow episode from BitRobot, Unitree and Hugging Face.");
  track(s, 6);
  s.addImage({ path: LOGO, x: 0.8, y: 1.6, w: 4.6, h: 4.6 });
  box(s, 6.2, 1.9, 6.5, 1.4, "github.com/awesomebytes/cppyy_kit", { fill: NAVY, t: { fontSize: 24, bold: true, color: WHITE, align: "center" } });
  box(s, 6.2, 3.6, 6.5, 1.4, "Dataset: HIW-500 G1 pillow episode, by BitRobot, Unitree and Hugging Face", { fill: GREY, t: { fontSize: 18, color: NAVY, bold: true, margin: 0.3 } });
  box(s, 6.2, 5.3, 6.5, 0.9, "ROSCon UK 2026", { fill: YEL, t: { fontSize: 22, bold: true, color: NAVY, align: "center" } });

  await pres.writeFile({ fileName: OUT });
  await applyTheme(OUT, THEME);
  console.log("written", OUT);
})();
