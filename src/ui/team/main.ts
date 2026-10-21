import '../../film/film.css';
import '../race/race.css';
import './team.css';
import { startTeam } from './page';

const root = document.querySelector('#app');
if (root instanceof HTMLElement) startTeam(root);
