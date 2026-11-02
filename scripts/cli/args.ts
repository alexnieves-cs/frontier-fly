// Option parsing for the headless CLI: `--name value`, `--name=value` and bare `--flag`.
// Every command lists its options, so a typo fails before a long flight starts.

export class UsageError extends Error {
  override name = 'UsageError';
}

export type OptionKind = 'value' | 'int' | 'flag';

export interface ParsedArgs {
  string(name: string): string | undefined;
  int(name: string): number | undefined;
  flag(name: string): boolean;
}

export function parseArgs(argv: readonly string[], known: Readonly<Record<string, OptionKind>>): ParsedArgs {
  const values = new Map<string, string>();
  const flags = new Set<string>();
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i]!;
    if (!a.startsWith('--') || a === '--') throw new UsageError(`unexpected argument "${a}"; options start with --`);
    const eq = a.indexOf('=');
    const name = a.slice(2, eq >= 0 ? eq : undefined);
    const kind = known[name];
    if (!kind) throw new UsageError(`unknown option --${name}; known: ${Object.keys(known).map((k) => `--${k}`).join(', ')}`);
    if (kind === 'flag') {
      if (eq >= 0) throw new UsageError(`--${name} takes no value`);
      flags.add(name);
      continue;
    }
    let value: string | undefined;
    if (eq >= 0) value = a.slice(eq + 1);
    else {
      value = argv[i + 1];
      if (value === undefined || value.startsWith('--')) throw new UsageError(`--${name} needs a value`);
      i++;
    }
    if (kind === 'int' && !/^-?\d+$/.test(value)) throw new UsageError(`--${name} must be a whole number, got "${value}"`);
    values.set(name, value);
  }
  return {
    string: (name) => values.get(name),
    int: (name) => (values.has(name) ? Number(values.get(name)) : undefined),
    flag: (name) => flags.has(name),
  };
}
