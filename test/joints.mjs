// Stub for the joints area. Loads the built module and passes. The joints task adds the real checks.
import { existsSync } from "node:fs";
import { fileURLToPath } from "node:url";

const entry = new URL("../dist/index.js", import.meta.url);
if (!existsSync(fileURLToPath(entry))) {
  console.log("joints: no dist/index.js build found. Skipping.");
  process.exit(0);
}
await import(entry.href);
console.log("ok joints: module loads");
