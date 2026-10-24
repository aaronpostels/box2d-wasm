// Counts the WebAssembly SIMD (v128) instructions in a .wasm file, so CI can prove the SIMD build really uses SIMD and the
// scalar build uses none. It decodes the code section instruction by instruction (a plain search for the 0xFD prefix byte
// would also hit immediates). No dependency, no wabt.
//   node scripts/audit_simd.mjs dist/box2d.wasm --min 1       exit 1 when fewer than 1 SIMD instruction
//   node scripts/audit_simd.mjs dist/box2d.scalar.wasm --max 0  exit 1 when more than 0
//   node scripts/audit_simd.mjs --selftest                     checks the decoder on tiny hand-built modules
import { readFileSync } from "node:fs";

class Reader {
  constructor(bytes, pos = 0) { this.b = bytes; this.p = pos; }
  u8() { return this.b[this.p++]; }
  // LEB128, unsigned or signed; values past 2^53 are not needed for lengths and indices, only skipped.
  leb() {
    let result = 0, shift = 0, byte;
    do {
      byte = this.u8();
      if (byte === undefined) throw new Error("unexpected end of module");
      result += (byte & 0x7f) * 2 ** shift;
      shift += 7;
    } while (byte & 0x80);
    return result;
  }
  skip(n) { this.p += n; }
}

// SIMD opcodes (after the 0xFD prefix) that carry immediates: memarg, 16 bytes, lane index, memarg + lane.
const simdImmediate = (op) => {
  if (op <= 0x0b || op === 0x5c || op === 0x5d) return "memarg"; // v128.load* / store, load32_zero, load64_zero
  if (op === 0x0c || op === 0x0d) return "bytes16"; // v128.const, i8x16.shuffle
  if (op >= 0x15 && op <= 0x22) return "lane"; // extract_lane / replace_lane
  if (op >= 0x54 && op <= 0x5b) return "memarg+lane"; // load/store lane
  return "none";
};

// Walks one function body (after its local declarations) and calls onSimd(op) for each v128 instruction.
function walkCode(r, end, onSimd) {
  const memarg = () => { const align = r.leb(); if (align >= 64) r.leb(); /* multi-memory index */ r.leb(); };
  while (r.p < end) {
    const op = r.u8();
    if (op === 0xfd) {
      const sop = r.leb();
      onSimd(sop);
      const imm = simdImmediate(sop);
      if (imm === "memarg") memarg();
      else if (imm === "bytes16") r.skip(16);
      else if (imm === "lane") r.skip(1);
      else if (imm === "memarg+lane") { memarg(); r.skip(1); }
    } else if (op === 0xfc) {
      const fop = r.leb();
      if (fop === 8) { r.leb(); r.skip(1); } // memory.init
      else if (fop === 9 || fop === 13 || (fop >= 15 && fop <= 17)) r.leb(); // data.drop, elem.drop, table.grow/size/fill
      else if (fop === 10) r.skip(2); // memory.copy
      else if (fop === 11) r.skip(1); // memory.fill
      else if (fop === 12 || fop === 14) { r.leb(); r.leb(); } // table.init, table.copy
    } else if (op === 0xfe) {
      const aop = r.leb();
      if (aop === 3) r.skip(1); // atomic.fence
      else memarg();
    } else if (op === 0x02 || op === 0x03 || op === 0x04) r.leb(); // block type
    else if (op === 0x0c || op === 0x0d || op === 0x10 || op === 0xd2 || (op >= 0x20 && op <= 0x26)) r.leb();
    else if (op === 0x0e) { const n = r.leb(); for (let i = 0; i <= n; i++) r.leb(); } // br_table
    else if (op === 0x11) { r.leb(); r.leb(); } // call_indirect
    else if (op === 0x1c) { const n = r.leb(); r.skip(n); } // typed select
    else if (op >= 0x28 && op <= 0x3e) memarg();
    else if (op === 0x3f || op === 0x40) r.skip(1);
    else if (op === 0x41 || op === 0x42) r.leb(); // i32.const, i64.const
    else if (op === 0x43) r.skip(4);
    else if (op === 0x44) r.skip(8);
    else if (op === 0xd0) r.leb(); // ref.null
    else if (op === 0x00 || op === 0x01 || op === 0x05 || op === 0x0b || op === 0x0f || op === 0x1a || op === 0x1b ||
             (op >= 0x45 && op <= 0xc4) || op === 0xd1) { /* no immediates */ }
    else throw new Error(`unsupported opcode 0x${op.toString(16)} at byte ${r.p - 1}`);
  }
}

/** Returns { total, distinct, byOp } for the SIMD instructions in a wasm binary. */
function auditSimd(bytes) {
  if (bytes[0] !== 0 || bytes[1] !== 0x61 || bytes[2] !== 0x73 || bytes[3] !== 0x6d) throw new Error("not a wasm module");
  const r = new Reader(bytes, 8);
  const byOp = new Map();
  let total = 0;
  const onSimd = (op) => { total++; byOp.set(op, (byOp.get(op) ?? 0) + 1); };
  while (r.p < bytes.length) {
    const id = r.u8();
    const size = r.leb();
    const end = r.p + size;
    if (id === 10) { // code section
      const count = r.leb();
      for (let i = 0; i < count; i++) {
        const bodySize = r.leb();
        const bodyEnd = r.p + bodySize;
        const groups = r.leb();
        for (let g = 0; g < groups; g++) { r.leb(); r.skip(1); } // local count, value type
        walkCode(r, bodyEnd, onSimd);
        r.p = bodyEnd;
      }
    }
    r.p = end;
  }
  return { total, distinct: byOp.size, byOp };
}

// ---- self test: modules built by hand ----

const leb = (n) => { const out = []; do { let b = n & 0x7f; n >>>= 7; if (n) b |= 0x80; out.push(b); } while (n); return out; };
const section = (id, content) => [id, ...leb(content.length), ...content];
// One function `() -> ()` whose body is `code`.
const moduleWith = (code) => {
  const body = [0, ...code, 0x0b]; // no locals, instructions, end
  return Uint8Array.from([
    0, 0x61, 0x73, 0x6d, 1, 0, 0, 0,
    ...section(1, [1, 0x60, 0, 0]), // type: () -> ()
    ...section(3, [1, 0]), // function 0 has type 0
    ...section(10, [1, ...leb(body.length), ...body]),
  ]);
};

function selftest() {
  const assertEq = (actual, expected, what) => { if (actual !== expected) throw new Error(`selftest: ${what}: got ${actual}, want ${expected}`); };
  // scalar: i32.const 0xFD (as a LEB byte 0xFD 0x01) must not count; i32.add; drop.
  assertEq(auditSimd(moduleWith([0x41, 0xfd, 0x01, 0x41, 0x01, 0x6a, 0x1a])).total, 0, "0xFD inside an i32.const immediate");
  // v128.const (16 bytes, one of them 0xFD), i32x4.add (0xFD 0xAE 0x01), i8x16.extract_lane_s 3, drop x2.
  const simd = [0xfd, 0x0c, ...new Array(15).fill(0), 0xfd, 0xfd, 0x0c, ...new Array(16).fill(0), 0xfd, 0xae, 0x01, 0x1a];
  const r = auditSimd(moduleWith(simd));
  assertEq(r.total, 3, "three SIMD instructions"); // const, const, add
  assertEq(r.distinct, 2, "two distinct opcodes");
  // v128.load with a memarg (align 4, offset 0x80 = two LEB bytes) then drop, plus an i8x16.shuffle.
  const mixed = [0x41, 0x00, 0xfd, 0x00, 0x04, 0x80, 0x01, 0x1a, 0xfd, 0x0d, ...new Array(16).fill(1), 0x1a];
  assertEq(auditSimd(moduleWith(mixed)).total, 2, "memarg and shuffle immediates are skipped");
  console.log("ok audit_simd selftest");
}

// ---- command line ----

const args = process.argv.slice(2);
if (args.includes("--selftest")) {
  selftest();
} else {
  const file = args.find((a) => !a.startsWith("--") && args[args.indexOf(a) - 1] !== "--min" && args[args.indexOf(a) - 1] !== "--max");
  if (!file) { console.error("usage: audit_simd.mjs <file.wasm> [--min N] [--max N] | --selftest"); process.exit(2); }
  const opt = (name) => { const i = args.indexOf(name); return i >= 0 ? Number(args[i + 1]) : undefined; };
  const { total, distinct } = auditSimd(readFileSync(file));
  console.log(`${file}: ${total} SIMD instructions, ${distinct} distinct opcodes`);
  const min = opt("--min"), max = opt("--max");
  if (min !== undefined && total < min) { console.error(`expected at least ${min}`); process.exit(1); }
  if (max !== undefined && total > max) { console.error(`expected at most ${max}`); process.exit(1); }
}
