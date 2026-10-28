import './how.css';
import { startHow } from './page';

const root = document.querySelector('#app');
if (root instanceof HTMLElement) startHow(root);
