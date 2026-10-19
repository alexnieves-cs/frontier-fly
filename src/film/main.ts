import './film.css';
import { startFilm } from './page';

const root = document.querySelector('#app');
if (root instanceof HTMLElement) void startFilm(root);
