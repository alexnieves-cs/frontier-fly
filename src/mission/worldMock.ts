import { siteLayout } from './site';
import type { WorldProvider, WorldSnapshot } from './world';

export const mockWorldProvider: WorldProvider = {
  id: 'mock-site',
  load(seed: number): WorldSnapshot {
    const site = siteLayout(seed);
    return {
      provider: this.id,
      seed: site.seed,
      siteName: site.siteName,
      footprints: site.footprints,
    };
  },
};
