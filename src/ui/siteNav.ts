/**
 * The one list of site pages and the one site header. Every page renders its navigation from here, so the
 * pages, their order, their names and the header look stay the same across the site.
 */
import './theme.css';

export const REPO_URL = 'https://github.com/alexnieves-cs/frontier-fly';

export type SitePage = 'home' | 'film' | 'demo' | 'race' | 'team' | 'atlas' | 'sim' | 'outdoor' | 'mission' | 'px4';

/** Named for what a visitor does there. `gap` starts the second group (tools) after a divider. */
const PAGES: ReadonlyArray<{ id: SitePage; href: string; label: string; title: string; gap?: boolean }> = [
  { id: 'home', href: './index.html', label: 'Home', title: 'Project overview and results' },
  { id: 'film', href: './film.html', label: 'Watch', title: 'A full mission, replayed in 3D' },
  { id: 'demo', href: './demo.html', label: 'Missions', title: 'Run a woods-to-building rescue mission' },
  { id: 'race', href: './race.html', label: 'Race', title: 'Four pilots search the same building live' },
  { id: 'team', href: './team.html', label: 'Team', title: 'Several drones search one building together' },
  { id: 'atlas', href: './atlas.html', label: 'Safety', title: 'How often it fails, how, and the flights to prove it' },
  { id: 'sim', href: './sim.html', label: 'Fly indoors', title: 'Fly or train the indoor drone yourself', gap: true },
  { id: 'outdoor', href: './outdoor.html', label: 'Fly outdoors', title: 'Fly the procedural forest site' },
  { id: 'mission', href: './mission.html', label: 'Plan', title: 'Plan a mission and export it' },
  { id: 'px4', href: './px4.html', label: 'PX4', title: 'PX4 SITL flies this simulator' },
];

/** The page links plus GitHub, with `aria-current` on the page being shown. */
export function siteNavLinks(current: SitePage): string {
  const links = PAGES.map(
    (p) =>
      `${p.gap ? '<span class="site-nav-gap" aria-hidden="true"></span>' : ''}` +
      `<a href="${p.href}" title="${p.title}"${p.id === current ? ' aria-current="page"' : ''}>${p.label}</a>`,
  );
  links.push(`<a href="${REPO_URL}" rel="noopener">GitHub</a>`);
  return links.join('');
}

let menuIds = 0;

/** The page links, folded behind a "Pages" button on narrow screens (see theme.css). */
export function siteNav(current: SitePage): string {
  const id = `site-nav-${++menuIds}`;
  return `<div class="site-menu">
    <button type="button" class="site-menu-toggle" aria-expanded="false" aria-controls="${id}" data-site-menu>Pages</button>
    <nav class="site-nav" id="${id}" aria-label="Pages">${siteNavLinks(current)}</nav>
  </div>`;
}

export interface SiteHeaderOptions {
  /** The page's name beside the wordmark. Rendered as the page's h1 unless `heading` is false. */
  page?: string;
  heading?: boolean;
  /** Page controls placed between the brand and the menu. */
  slot?: string;
  /** Put the controls on their own row under the brand and menu, for pages with many of them. */
  slotRow?: boolean;
  className?: string;
}

/** The shared header: logo and wordmark (linking home), page name, page controls, menu. */
export function siteHeader(current: SitePage, opts: SiteHeaderOptions = {}): string {
  const tag = opts.heading === false ? 'span' : 'h1';
  const page = opts.page ? `<${tag} class="site-page">${opts.page}</${tag}>` : '';
  const slot = opts.slot ? `<div class="site-slot">${opts.slot}</div>` : '';
  const classes = ['site-header', opts.slotRow ? 'slot-row' : '', opts.className ?? ''].filter(Boolean).join(' ');
  return `<header class="${classes}">
    <a class="site-brand" href="./index.html"><span class="site-mark" aria-hidden="true"></span><span class="site-word">frontier-fly</span></a>
    ${page}${slot}${siteNav(current)}
  </header>`;
}

/** True when the visitor asked the system for less motion. */
export function prefersReducedMotion(): boolean {
  return typeof matchMedia === 'function' && matchMedia('(prefers-reduced-motion: reduce)').matches;
}

function closeMenus(except?: Element): void {
  for (const menu of document.querySelectorAll('.site-menu.open')) {
    if (menu === except) continue;
    menu.classList.remove('open');
    menu.querySelector('[data-site-menu]')?.setAttribute('aria-expanded', 'false');
  }
}

// One listener for every page's menu, so pages render the markup and nothing else.
if (typeof document !== 'undefined') {
  document.addEventListener('click', (event) => {
    const target = event.target instanceof Element ? event.target : null;
    const toggle = target?.closest('[data-site-menu]');
    if (toggle) {
      const menu = toggle.closest('.site-menu');
      if (!menu) return;
      const open = !menu.classList.contains('open');
      closeMenus(menu);
      menu.classList.toggle('open', open);
      toggle.setAttribute('aria-expanded', String(open));
      if (open) menu.querySelector<HTMLElement>('.site-nav a')?.focus();
      return;
    }
    if (!target?.closest('.site-menu')) closeMenus();
  });
  document.addEventListener('keydown', (event) => {
    if (event.key !== 'Escape') return;
    const open = document.querySelector('.site-menu.open');
    if (!open) return;
    closeMenus();
    open.querySelector<HTMLElement>('[data-site-menu]')?.focus();
  });
}
