const SCENES = new Set(['night', 'hangar']);
export const SCENE_STORAGE_KEY = 'robot-lab-scene';

// Presentation only: this module deliberately has no coordinator, robot,
// physics clock, camera or task dependency.
export function createThemeController({workcell, root, buttons, storageProvider = () => window.localStorage}) {
  const controls = Array.from(buttons);
  let storage, remembered, disposed = false;
  try { storage = storageProvider(); remembered = storage?.getItem(SCENE_STORAGE_KEY); } catch { /* Storage is optional. */ }
  let activeId = SCENES.has(remembered) ? remembered : 'night';
  function setScene(id, {persist = true} = {}) {
    if (disposed || !SCENES.has(id)) return false;
    const result = workcell.setScene(id);
    if (result === false) return false;
    activeId = id;
    root.dataset.scene = id;
    root.ownerDocument?.querySelector('meta[name="theme-color"]')?.setAttribute('content', id === 'night' ? '#1b2e3d' : '#273541');
    for (const button of controls) button.setAttribute('aria-pressed', String(button.dataset.sceneSwitch === id));
    if (persist) { try { storage?.setItem(SCENE_STORAGE_KEY, id); } catch { /* Keep the chosen scene in memory. */ } }
    return true;
  }
  const listeners = controls.map(button => {
    const handler = () => setScene(button.dataset.sceneSwitch);
    button.addEventListener('click', handler);
    return [button, handler];
  });
  setScene(activeId, {persist:false});
  return {
    get activeId() { return activeId; },
    setScene,
    dispose() {
      if (disposed) return;
      for (const [button, handler] of listeners) button.removeEventListener('click', handler);
      disposed = true;
    },
  };
}
