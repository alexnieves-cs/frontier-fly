import './atlas.css';
import { startAtlas } from './page';

const root = document.querySelector('#app');
if (root instanceof HTMLElement) startAtlas(root);
