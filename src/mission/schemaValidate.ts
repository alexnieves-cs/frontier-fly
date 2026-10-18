import missionSchemaJson from './mission.schema.json';

export interface JsonSchema {
  $id?: string;
  $schema?: string;
  $ref?: string;
  $defs?: Record<string, JsonSchema>;
  title?: string;
  description?: string;
  type?: string | string[];
  const?: string | number | boolean | null;
  enum?: (string | number | boolean | null)[];
  properties?: Record<string, JsonSchema>;
  required?: string[];
  additionalProperties?: boolean;
  items?: JsonSchema;
  minLength?: number;
  maxLength?: number;
  pattern?: string;
  minimum?: number;
  maximum?: number;
  exclusiveMinimum?: number;
  exclusiveMaximum?: number;
  minItems?: number;
  maxItems?: number;
}

export interface SchemaIssue {
  path: string;
  keyword: string;
  message: string;
}

export const missionSchema = missionSchemaJson as unknown as JsonSchema;

function isRecord(data: unknown): data is Record<string, unknown> {
  return typeof data === 'object' && data !== null && !Array.isArray(data);
}

function matchesType(data: unknown, type: string): boolean {
  switch (type) {
    case 'string':
      return typeof data === 'string';
    case 'number':
      return typeof data === 'number' && Number.isFinite(data);
    case 'integer':
      return typeof data === 'number' && Number.isInteger(data);
    case 'boolean':
      return typeof data === 'boolean';
    case 'null':
      return data === null;
    case 'object':
      return isRecord(data);
    case 'array':
      return Array.isArray(data);
    default:
      return false;
  }
}

function joinPath(parent: string, key: string, index: boolean): string {
  if (!parent) return index ? `[${key}]` : key;
  return index ? `${parent}[${key}]` : `${parent}.${key}`;
}

function walk(data: unknown, schema: JsonSchema, path: string, root: JsonSchema, issues: SchemaIssue[]): void {
  if (schema.$ref) {
    const prefix = '#/$defs/';
    const name = schema.$ref.startsWith(prefix) ? schema.$ref.slice(prefix.length) : '';
    const next = name && root.$defs ? root.$defs[name] : undefined;
    if (!next) {
      issues.push({ path, keyword: '$ref', message: `Unknown schema reference ${schema.$ref}.` });
      return;
    }
    walk(data, next, path, root, issues);
    return;
  }

  if (schema.type) {
    const types = Array.isArray(schema.type) ? schema.type : [schema.type];
    if (!types.some((type) => matchesType(data, type))) {
      const message = types.length === 1 && types[0] === 'integer' ? 'Must be an integer.' : `Must be ${types.join(' or ')}.`;
      issues.push({ path, keyword: 'type', message });
      return;
    }
  }

  if ('const' in schema && schema.const !== undefined && data !== schema.const) {
    issues.push({ path, keyword: 'const', message: `Must be ${JSON.stringify(schema.const)}.` });
    return;
  }

  if (schema.enum && !schema.enum.some((item) => item === data)) {
    issues.push({
      path,
      keyword: 'enum',
      message: `Must be one of ${schema.enum.map((item) => String(item)).join(', ')}.`,
    });
    return;
  }

  if (typeof data === 'string') {
    if (schema.minLength != null && data.length < schema.minLength) {
      issues.push({ path, keyword: 'minLength', message: 'Must not be empty.' });
    }
    if (schema.maxLength != null && data.length > schema.maxLength) {
      issues.push({ path, keyword: 'maxLength', message: `Must be at most ${schema.maxLength} characters.` });
    }
    if (schema.pattern && !new RegExp(schema.pattern).test(data)) {
      issues.push({ path, keyword: 'pattern', message: 'Use letters, numbers, hyphens, or underscores.' });
    }
  }

  if (typeof data === 'number') {
    if (schema.minimum != null && data < schema.minimum) {
      issues.push({ path, keyword: 'minimum', message: `Must be at least ${schema.minimum}.` });
    }
    if (schema.maximum != null && data > schema.maximum) {
      issues.push({ path, keyword: 'maximum', message: `Must be at most ${schema.maximum}.` });
    }
    if (schema.exclusiveMinimum != null && data <= schema.exclusiveMinimum) {
      issues.push({ path, keyword: 'exclusiveMinimum', message: `Must be greater than ${schema.exclusiveMinimum}.` });
    }
    if (schema.exclusiveMaximum != null && data >= schema.exclusiveMaximum) {
      issues.push({ path, keyword: 'exclusiveMaximum', message: `Must be less than ${schema.exclusiveMaximum}.` });
    }
  }

  if (Array.isArray(data)) {
    if (schema.minItems != null && data.length < schema.minItems) {
      issues.push({ path, keyword: 'minItems', message: `Must have at least ${schema.minItems}.` });
    }
    if (schema.maxItems != null && data.length > schema.maxItems) {
      issues.push({ path, keyword: 'maxItems', message: `Must have at most ${schema.maxItems}.` });
    }
    if (schema.items) {
      const itemSchema = schema.items;
      data.forEach((item, index) => walk(item, itemSchema, joinPath(path, String(index), true), root, issues));
    }
    return;
  }

  if (isRecord(data) && schema.properties) {
    const required = schema.required ?? [];
    for (const key of required) {
      if (!(key in data)) issues.push({ path: joinPath(path, key, false), keyword: 'required', message: 'Required.' });
    }
    if (schema.additionalProperties === false) {
      for (const key of Object.keys(data)) {
        if (!schema.properties[key]) {
          issues.push({ path: joinPath(path, key, false), keyword: 'additionalProperties', message: 'Unknown field.' });
        }
      }
    }
    for (const [key, propSchema] of Object.entries(schema.properties)) {
      if (key in data) walk(data[key], propSchema, joinPath(path, key, false), root, issues);
    }
  }
}

/** Validate `data` against the mission JSON Schema. Supports the keyword subset used by v1. */
export function validateJsonSchema(data: unknown, schema: JsonSchema = missionSchema): SchemaIssue[] {
  const issues: SchemaIssue[] = [];
  walk(data, schema, '', schema, issues);
  return issues;
}
