// Stub for the queries area. Loads the built module and passes. The queries task adds the real checks.
import { existsSync } from "node:fs";
import { fileURLToPath } from "node:url";

const entry = new URL("../dist/index.js", import.meta.url);
if (!existsSync(fileURLToPath(entry))) {
  console.log("queries: no dist/index.js build found. Skipping.");
  process.exit(0);
}
await import(entry.href);
console.log("ok queries: module loads");
