import { startApp } from './ui/app';

const root = document.querySelector('#app');
if (root instanceof HTMLElement) {
  void startApp(root);
}
