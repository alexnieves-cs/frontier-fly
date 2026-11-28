import './duo.css';
import { startDuo } from './page';

const root = document.querySelector('#app');
if (root instanceof HTMLElement) startDuo(root);
