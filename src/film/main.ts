import './film.css';
import { startFilm } from './page';

document.documentElement.classList.add('film-page');

const root = document.querySelector('#app');
if (root instanceof HTMLElement) void startFilm(root);
