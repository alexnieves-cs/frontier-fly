/**
 * Where e2e screenshots go. A normal run writes them to test-results/media so checking a change leaves the tree
 * clean; `UPDATE_MEDIA=1 npm run test:e2e` refreshes the committed images in docs/media.
 */
export const out = process.env.UPDATE_MEDIA === '1' ? 'docs/media' : 'test-results/media';
