import '../../film/film.css';
import './race.css';
import '../inspector/inspector.css';
import { startRace } from './page';

const root = document.querySelector('#app');
if (root instanceof HTMLElement) startRace(root);
