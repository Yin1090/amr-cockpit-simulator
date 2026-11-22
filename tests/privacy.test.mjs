import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT_DIR = path.resolve(__dirname, '..');

function collectFiles(dir) {
  const out = [];
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    if (entry.name === 'node_modules' || entry.name === '.git') continue;
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      out.push(...collectFiles(full));
    } else if (/\.(html|css|js|mjs|json|csv|md|txt)$/i.test(entry.name)) {
      out.push(full);
    }
  }
  return out;
}

// Generic security/sanitization checks (character classes prevent self-matching without storing PII literals)
const FORBIDDEN_PATTERNS = [
  { name: 'private-ipv4-10', re: /\b1[0]\.\d{1,3}\.\d{1,3}\.\d{1,3}\b/ },
  { name: 'private-ipv4-192', re: /\b19[2]\.168\.\d{1,3}\.\d{1,3}\b/ },
  { name: 'private-ipv4-172', re: /\b17[2]\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3}\b/ },
  { name: 'consumer-email', re: /[a-zA-Z0-9._%+-]+@(?:[g]mail|[y]ahoo|[o]utlook|[h]otmail)\.[a-zA-Z]{2,}/ },
  { name: 'bot-token', re: /\b\d{8,10}:[A-Za-z0-9_-]{35}\b/ },
  { name: 'github-token', re: /\b(?:[g]hp_[A-Za-z0-9]{20,}|[g]ithub_pat_[A-Za-z0-9_]{20,})\b/ },
  { name: 'oauth-client-secret', re: /[c]lient_secret/i },
  { name: 'credential-assignment', re: /\b(?:[w]ifi_pass|[s]sid|[p]sk|[p]assword)\s*=\s*['"][^'"]+['"]/i },
  { name: 'forbidden-legacy-ext', re: /\.(?:a[i]a|b[k]y|s[c]m)\b/i },
  { name: 'external-cdn', re: /fonts\.[g]oogleapis\.com|fonts\.[g]static\.com|cdn\.[t]ailwindcss\.com|[u]npkg\.com|cdn\.[j]sdelivr\.net|cdnjs\.[c]loudflare\.com/i },
];

test('all simulator files satisfy zero-tolerance privacy and zero-CDN policy', () => {
  const files = collectFiles(ROOT_DIR);
  assert.ok(files.length >= 10, 'Expected at least 10 project files to scan');

  const violations = [];
  for (const file of files) {
    const content = fs.readFileSync(file, 'utf8');
    const rel = path.relative(ROOT_DIR, file);
    for (const rule of FORBIDDEN_PATTERNS) {
      const m = content.match(rule.re);
      if (m) {
        violations.push(`${rel} matched [${rule.name}]: ${m[0]}`);
      }
    }
    if (rel === 'index.html') {
      assert.equal(/https?:\/\//i.test(content), false, 'index.html must not contain http/https URLs');
    }
  }

  assert.deepEqual(violations, [], `Privacy/CDN violations found:\n${violations.join('\n')}`);
});
