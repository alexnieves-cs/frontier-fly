import './mission.css';
import { mountMissionEditor } from './page';

const root = document.querySelector('#app');
if (root instanceof HTMLElement) mountMissionEditor(root);
