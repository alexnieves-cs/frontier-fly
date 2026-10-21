/**
 * The one list of site pages. Every page renders its navigation from here, so the pages,
 * their order and their names stay the same across the site.
 */
import './siteNav.css';

export const REPO_URL = 'https://github.com/alexnieves-cs/frontier-fly';

export type SitePage = 'home' | 'film' | 'demo' | 'race' | 'team' | 'sim' | 'mission' | 'outdoor' | 'px4';

const PAGES: ReadonlyArray<{ id: SitePage; href: string; label: string; title: string }> = [
  { id: 'home', href: './index.html', label: 'Home', title: 'Project overview and results' },
  { id: 'film', href: './film.html', label: 'Film', title: 'A full mission, replayed in 3D' },
  { id: 'demo', href: './demo.html', label: 'Missions', title: 'Autonomous woods-to-building missions' },
  { id: 'race', href: './race.html', label: 'Race', title: 'Four pilots search the same building live' },
  { id: 'team', href: './team.html', label: 'Team', title: 'Several drones search one building together' },
  { id: 'sim', href: './sim.html', label: 'Sim', title: 'Fly or train the indoor drone yourself' },
  { id: 'mission', href: './mission.html', label: 'Editor', title: 'Plan a mission and export it' },
  { id: 'outdoor', href: './outdoor.html', label: 'Outdoor', title: 'Fly the procedural forest site' },
  { id: 'px4', href: './px4.html', label: 'PX4', title: 'PX4 SITL flies this simulator' },
];

/** The page links plus GitHub, with `aria-current` on the page being shown. */
export function siteNavLinks(current: SitePage): string {
  const links = PAGES.map(
    (p) => `<a href="${p.href}" title="${p.title}"${p.id === current ? ' aria-current="page"' : ''}>${p.label}</a>`,
  );
  links.push(`<a href="${REPO_URL}" rel="noopener">GitHub</a>`);
  return links.join('');
}

/** A self-styled nav bar for pages that have no header nav of their own (see siteNav.css). */
export function siteNav(current: SitePage): string {
  return `<nav class="site-nav" aria-label="Pages">${siteNavLinks(current)}</nav>`;
}
