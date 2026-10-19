import './demo.css';
import { startDemo } from './page';

const root = document.querySelector('#app');
if (root instanceof HTMLElement) startDemo(root);
