'use strict';

const { contextBridge, ipcRenderer } = require('electron');

// Expose a safe bridge so the renderer can use native file dialogs
// instead of the browser's limited file picker
contextBridge.exposeInMainWorld('electronAPI', {
  // Save a file via native OS dialog
  saveFile: (defaultName, content) =>
    ipcRenderer.invoke('save-file', { defaultName, content }),

  // Open a file via native OS dialog — returns { ok, filePath, content }
  openFile: (filters) =>
    ipcRenderer.invoke('open-file', { filters }),

  // Let the renderer know it's running inside Electron
  isElectron: true,
  platform: process.platform,  // 'darwin' | 'win32' | 'linux'
});
