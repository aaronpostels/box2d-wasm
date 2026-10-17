// SPDX-License-Identifier: MIT

import { RawBox2DModule } from "./types.js";
import { Box2DWorld } from "./world.js";

export * from "./types.js";
export * from "./world.js";

export type Box2DModuleFactory = (options?: Record<string, unknown>) => Promise<RawBox2DModule>;

/**
 * Initializes Box2D WASM instance using the generated Emscripten module factory.
 */
export async function initBox2D(
    factoryOrModule?: Box2DModuleFactory | RawBox2DModule,
    options?: Record<string, unknown>
): Promise<RawBox2DModule> {
    if (!factoryOrModule) {
        try {
            // @ts-ignore
            const mod = await import("./box2d.js");
            const factory = (mod.default ?? mod) as Box2DModuleFactory;
            return await factory(options);
        } catch (err) {
            throw new Error(
                "Could not load './box2d.js'. Please build WASM first or pass the module factory explicitly: initBox2D(Box2DModuleFactory).",
                { cause: err }
            );
        }
    }

    if (typeof factoryOrModule === "function") {
        return await factoryOrModule(options);
    }

    return factoryOrModule;
}
