// Stub for the filters area. Loads the built module and passes. The filters task adds the real checks.
import { existsSync } from "node:fs";
import { fileURLToPath } from "node:url";

const entry = new URL("../dist/index.js", import.meta.url);
if (!existsSync(fileURLToPath(entry))) {
  console.log("filters: no dist/index.js build found. Skipping.");
  process.exit(0);
}
await import(entry.href);
console.log("ok filters: module loads");
