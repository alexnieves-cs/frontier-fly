import { describe, expect, it } from 'vitest';
import { errorMessage, isWebglError } from '../../src/ui/startError';

describe('start errors', () => {
  it('recognises three.js failing to get a WebGL context', () => {
    expect(isWebglError(new Error('Error creating WebGL context.'))).toBe(true);
    expect(isWebglError('Error creating WebGL context.')).toBe(true);
  });

  it('leaves other failures to show their own message', () => {
    expect(isWebglError(new Error('Failed to fetch plans.json'))).toBe(false);
    expect(errorMessage(new Error('Failed to fetch plans.json'))).toBe('Failed to fetch plans.json');
  });
});
