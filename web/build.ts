import { copyFile, mkdir, readdir, readFile, rm, writeFile } from 'node:fs/promises';
import { extname, relative, resolve } from 'node:path';
import { stripTypeScriptTypes } from 'node:module';

const root = resolve(import.meta.dirname, '..');
const source = resolve(root, 'web');
const output = resolve(source, 'dist');

async function entries(folder: string): Promise<string[]> {
  const children = await readdir(folder, { withFileTypes: true });
  const paths = await Promise.all(children.map(async child => {
    const path = resolve(folder, child.name);
    return child.isDirectory() ? entries(path) : [path];
  }));
  return paths.flat();
}

async function compileModule(file: string): Promise<void> {
  const contents = await readFile(file, 'utf8');
  const javascript = stripTypeScriptTypes(contents, { mode: 'transform' });
  const browserImports = javascript.replace(/\.ts(?=['"])/g, '.js');
  const destination = resolve(output, relative(source, file)).replace(/\.ts$/, '.js');
  await mkdir(resolve(destination, '..'), { recursive: true });
  await writeFile(destination, browserImports);
}

async function copyAsset(file: string): Promise<void> {
  const destination = resolve(output, relative(source, file));
  await mkdir(resolve(destination, '..'), { recursive: true });
  await copyFile(file, destination);
}

async function build(): Promise<void> {
  await rm(output, { recursive: true, force: true });
  await mkdir(output, { recursive: true });
  const files = await entries(source);
  const modules = files.filter(file => file.includes('/scripts/'))
    .filter(file => extname(file) === '.ts' && !file.endsWith('.d.ts'));
  const assets = files.filter(file => !file.includes('/dist/'))
    .filter(file => ['.html', '.css', '.wasm', '.js'].includes(extname(file)))
    .filter(file => extname(file) !== '.wasm' || file.includes('/native/'));
  await Promise.all([...modules.map(compileModule), ...assets.map(copyAsset)]);
  console.log(`Built ${modules.length} TypeScript modules into web/dist.`);
}

await build();
