'use strict';

const { app, BrowserWindow, ipcMain, dialog, shell } = require('electron');
const path = require('path');
const fs   = require('fs');

// ── Web Serial API flags ───────────────────────────────────────────────────────
// Do NOT use 'enable-experimental-web-platform-features' — on Electron 20+ it
// causes requestPort() to show the Bluetooth chooser instead of serial.
// The targeted flag below enables only the Serial API in the renderer.
app.commandLine.appendSwitch('enable-features', 'ElectronSerialChooser');

// ── Keep a global reference so window isn't GC'd ─────────────────────────────
let mainWindow = null;

function createWindow() {
  mainWindow = new BrowserWindow({
    width:  1280,
    height: 820,
    minWidth:  900,
    minHeight: 600,
    title: 'OBD2 CAN Logger',
    backgroundColor: '#0e1117',
    webPreferences: {
      preload:          path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration:  false,
      webSecurity:      true,
    },
    show: false,
  });

  const htmlPath = path.join(__dirname, 'app.html');
  mainWindow.loadFile(htmlPath);

  // ── Serial port permissions ───────────────────────────────────────────────────
  // setPermissionCheckHandler: allow the Serial API to be used at all.
  mainWindow.webContents.session.setPermissionCheckHandler(
    (_wc, permission) => permission === 'serial'
  );

  // setDevicePermissionHandler: auto-approve re-use of a previously selected port
  // without prompting again (e.g. on reconnect).
  mainWindow.webContents.session.setDevicePermissionHandler(
    (details) => details.deviceType === 'serial'
  );

  // ── Port chooser ──────────────────────────────────────────────────────────────
  // In Electron 20+, navigator.serial.requestPort() does NOT show a built-in
  // chooser — the main process MUST handle 'select-serial-port' and call back
  // with the chosen portId.  Without this handler Electron falls through to the
  // system device picker which on macOS shows a Bluetooth dialog.
  //
  // We show a native dialog listing the available ports.  The user picks one and
  // we call back with its portId so the renderer's requestPort() resolves.
  mainWindow.webContents.session.on(
    'select-serial-port',
    (event, portList, _webContents, callback) => {
      // Prevent Electron's default (broken) fallback behaviour
      event.preventDefault();

      if (!portList || portList.length === 0) {
        // No ports found — send back empty string to reject requestPort()
        // The renderer will show "No ports available" via its own error handler.
        callback('');
        return;
      }

      if (portList.length === 1) {
        // Only one port — select it automatically, no dialog needed
        callback(portList[0].portId);
        return;
      }

      // Multiple ports — show a native selection dialog
      const buttons = portList.map(p => p.displayName || p.portName || p.portId);
      buttons.push('Cancel');

      dialog.showMessageBox(mainWindow, {
        type:    'question',
        title:   'Select Serial Port',
        message: 'Choose the OBD2 CAN Logger port:',
        buttons,
        cancelId: buttons.length - 1,
      }).then(({ response }) => {
        if (response === buttons.length - 1) {
          callback(''); // cancelled
        } else {
          callback(portList[response].portId);
        }
      });
    }
  );

  // ── Serial port added/removed events (optional — useful for future UI) ────────
  mainWindow.webContents.session.on('serial-port-added', (_event, _port) => {
    // Could notify the renderer to refresh a port list UI
  });
  mainWindow.webContents.session.on('serial-port-removed', (_event, _port) => {
    // Could notify the renderer that a port disappeared
  });

  // ── Show window once DOM is ready ─────────────────────────────────────────────
  mainWindow.once('ready-to-show', () => mainWindow.show());

  // ── Open external links in default browser ────────────────────────────────────
  mainWindow.webContents.setWindowOpenHandler(({ url }) => {
    shell.openExternal(url);
    return { action: 'deny' };
  });

  mainWindow.on('closed', () => { mainWindow = null; });
}

// ── App lifecycle ─────────────────────────────────────────────────────────────
app.whenReady().then(() => {
  createWindow();
  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});

// ── IPC: save file ────────────────────────────────────────────────────────────
ipcMain.handle('save-file', async (event, { defaultName, content }) => {
  const { canceled, filePath } = await dialog.showSaveDialog(mainWindow, {
    defaultPath: defaultName,
    filters: [
      { name: 'JSON',      extensions: ['json'] },
      { name: 'CSV',       extensions: ['csv']  },
      { name: 'All Files', extensions: ['*']    },
    ],
  });
  if (canceled || !filePath) return { ok: false };
  fs.writeFileSync(filePath, content, 'utf8');
  return { ok: true, filePath };
});

// ── IPC: open file ────────────────────────────────────────────────────────────
ipcMain.handle('open-file', async (event, { filters }) => {
  const { canceled, filePaths } = await dialog.showOpenDialog(mainWindow, {
    properties: ['openFile'],
    filters: filters || [{ name: 'All Files', extensions: ['*'] }],
  });
  if (canceled || !filePaths.length) return { ok: false };
  const content = fs.readFileSync(filePaths[0], 'utf8');
  return { ok: true, filePath: filePaths[0], content };
});
