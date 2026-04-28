'use strict';

const { app, BrowserWindow, ipcMain, dialog, shell } = require('electron');
const path = require('path');
const fs   = require('fs');  // still used by IPC save/open handlers

// ── Enable Web Serial API (experimental in Electron) ──────────────────────────
app.commandLine.appendSwitch('enable-experimental-web-platform-features');
// Allow serial port access without user gesture in some Electron versions
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
      preload:               path.join(__dirname, 'preload.js'),
      contextIsolation:      true,
      nodeIntegration:       false,
      // Serial API needs these
      enableBlinkFeatures:   'Serial',
      webSecurity:           true,
    },
    // Show window only once ready to avoid white flash
    show: false,
  });

  // ── Load the app HTML ──────────────────────────────────────────────────────
  // app.html lives in src/ — bundled directly via the "files" glob in electron-builder.json
  // __dirname is .../resources/app/src in production, and desktop/src in dev
  const htmlPath = path.join(__dirname, 'app.html');

  mainWindow.loadFile(htmlPath);

  // ── Grant serial port permissions ────────────────────────────────────────────
  // Do NOT handle 'select-serial-port' — letting Electron show its own port
  // picker is more reliable than auto-selecting, which silently fails when the
  // port list is empty or the wrong port is picked.
  //
  // setPermissionCheckHandler:  called synchronously to decide if the API is
  //   allowed at all for this origin. Return true for serial.
  // setDevicePermissionHandler: called when the app tries to use a previously
  //   granted device. Return true to allow re-use without prompting again.
  mainWindow.webContents.session.setPermissionCheckHandler(
    (_wc, permission) => permission === 'serial'
  );

  mainWindow.webContents.session.setDevicePermissionHandler(
    (details) => details.deviceType === 'serial'
  );

  // In Electron 20+ the serial chooser is shown by the renderer's
  // navigator.serial.requestPort() call — no extra main-process wiring needed.

  // ── Show window once DOM is ready ─────────────────────────────────────────
  mainWindow.once('ready-to-show', () => {
    mainWindow.show();
  });

  // ── Open external links in default browser ────────────────────────────────
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

// ── IPC: save file dialog (for exporting map data / logs) ────────────────────
ipcMain.handle('save-file', async (event, { defaultName, content }) => {
  const { canceled, filePath } = await dialog.showSaveDialog(mainWindow, {
    defaultPath: defaultName,
    filters: [
      { name: 'JSON',     extensions: ['json'] },
      { name: 'CSV',      extensions: ['csv']  },
      { name: 'All Files',extensions: ['*']    },
    ],
  });
  if (canceled || !filePath) return { ok: false };
  fs.writeFileSync(filePath, content, 'utf8');
  return { ok: true, filePath };
});

// ── IPC: open file dialog (for loading DBC / CSV logs) ───────────────────────
ipcMain.handle('open-file', async (event, { filters }) => {
  const { canceled, filePaths } = await dialog.showOpenDialog(mainWindow, {
    properties: ['openFile'],
    filters: filters || [{ name: 'All Files', extensions: ['*'] }],
  });
  if (canceled || !filePaths.length) return { ok: false };
  const content = fs.readFileSync(filePaths[0], 'utf8');
  return { ok: true, filePath: filePaths[0], content };
});
