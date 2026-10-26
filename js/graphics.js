// ═══════════════════════════════════════════════════════════════════
// RobSimul Simple – Grafikqualität
//   standard : bisherige Darstellung (Phong, ohne Schatten) – schnell, für schwache Rechner
//   high     : PBR (MeshPhysicalMaterial), Studio-Umgebung (RoomEnvironment), weiche Schatten,
//              ACES-Tonemapping, sRGB-Ausgabe
//   max      : wie high + Ambient Occlusion (SSAO) + FXAA (Post-Processing)
// Umschalten: Toolbar „Grafik“, URL ?quality=…, RobSim.cmd('quality', …)
// Glatte STL-Kanten (Normalen mit Knickwinkel) gelten in allen Stufen (js/simulator.js, _smoothNormals).
// ═══════════════════════════════════════════════════════════════════
(function () {
  'use strict';
  var LEVELS = ['standard', 'high', 'max'], LBL = { standard: 'Standard', high: 'Hoch', max: 'Maximal' };
  var KEY = 'robsimSimple.quality';
  var quality = 'high';
  try {
    var qp = new URLSearchParams(location.search).get('quality') || localStorage.getItem(KEY);
    if (LEVELS.indexOf(qp) >= 0) quality = qp;
  } catch (e) {}

  // ── Lichter der Szene (aus simulator.js) ─────────────────────────
  var amb = null; scene.children.forEach(function (o) { if (o.isAmbientLight) amb = o; });
  var SUN_DIR = sun.position.clone().normalize();
  var LIGHT_STD = { amb: amb ? amb.intensity : 0.85, sun: sun.intensity, fill: fill.intensity, sunPos: sun.position.clone() };
  var LIGHT_HI = { amb: 0.1, sun: 1.25, fill: 0.2 };
  var EXPOSURE = 0.9, ENV_INTENSITY = 0.6;

  sun.shadow.mapSize.set(4096, 4096);
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 1.2;
  sun.shadow.radius = 3;
  scene.add(sun.target);

  // Boden, der nur Schatten empfängt (unsichtbar sonst)
  var ground = new THREE.Mesh(new THREE.PlaneGeometry(60000, 60000),
    new THREE.ShadowMaterial({ opacity: 0.28, transparent: true, depthWrite: false }));
  ground.position.z = -0.5; ground.receiveShadow = true; ground.renderOrder = -1;
  ground.userData.gfx = true; ground.visible = false;
  scene.add(ground);

  // Studio-Umgebung (ohne Download, keine HDR-Datei nötig)
  var envTex = null;
  function ensureEnv() {
    if (envTex || !THREE.RoomEnvironment) return envTex;
    var pmrem = new THREE.PMREMGenerator(renderer);
    envTex = pmrem.fromScene(new THREE.RoomEnvironment(), 0.04).texture;
    pmrem.dispose();
    return envTex;
  }

  // ── Materialien ──────────────────────────────────────────────────
  // Phong-Materialien (Roboterglieder, Podest, Werkzeug) bekommen in high/max ein PBR-Gegenstück.
  function physFrom(m) {
    var c = (m.userData.c0 || m.color).clone(), hsl = {}; c.getHSL(hsl);
    var metal = hsl.s < 0.12 && hsl.l < 0.55;          // grau/dunkel = Guss, Motoren, Flansch
    var p = new THREE.MeshPhysicalMaterial({
      color: c,
      metalness: metal ? 0.6 : 0.05,
      roughness: metal ? 0.36 : 0.42,
      clearcoat: metal ? 0.1 : 0.45,                   // Lackschicht auf lackierten Teilen
      clearcoatRoughness: 0.2,
      envMapIntensity: ENV_INTENSITY,
      transparent: m.transparent, opacity: m.opacity, side: m.side
    });
    p.userData.fromStd = true;
    return p;
  }
  function s2l(v) { return v <= 0.04045 ? v * 0.0773993808 : Math.pow(v * 0.9478672986 + 0.0521327014, 2.4); }

  var casterKey = '';
  function prepare(hi) {
    var keyParts = [];
    scene.traverse(function (o) {
      if (o.userData.gfx) return;
      // Material tauschen
      if (o.isMesh && o.material && !Array.isArray(o.material)) {
        if (o.material.isMeshPhongMaterial && !o.userData.matStd) { o.userData.matStd = o.material; }
        var ms = o.userData.matStd;
        if (ms) {
          if (hi && !o.userData.matPhys) o.userData.matPhys = physFrom(ms);
          var mp = o.userData.matPhys;
          if (mp) { mp.opacity = ms.opacity; mp.transparent = ms.transparent; mp.visible = ms.visible; }
          o.material = hi ? mp : ms;
          o.castShadow = hi; o.receiveShadow = hi;
          if (hi && o.visible) keyParts.push(o.id);
        }
      }
      // Farben: standard = Original; high/max = linear gerechnet, Ausgabe sRGB
      var mode = hi ? 'h' : 's';
      var mats = o.material ? (Array.isArray(o.material) ? o.material : [o.material]) : [];
      mats.forEach(function (m) {
        if (!m || !m.color) return;
        var u = m.userData;
        if (!u.c0) { u.c0 = m.color.clone(); if (m.emissive) u.e0 = m.emissive.clone(); }
        if (u.cm === mode) return;
        var pbr = m.isMeshPhysicalMaterial || m.isMeshStandardMaterial || m.isMeshPhongMaterial || m.isMeshLambertMaterial;
        m.color.copy(u.c0); if (hi) m.color.convertSRGBToLinear();
        if (u.e0) { m.emissive.copy(u.e0); if (hi) m.emissive.convertSRGBToLinear(); }
        // Linien, Punkte, Beschriftungen, Marker: exakte Farben, kein Tonemapping
        if (!pbr) m.toneMapped = false;
        if (m.map) { m.map.encoding = hi ? THREE.sRGBEncoding : THREE.LinearEncoding; m.map.needsUpdate = true; }
        u.cm = mode; m.needsUpdate = true;
      });
      var g = o.geometry;
      if (g && g.attributes && g.attributes.color) {
        var ud = g.userData, a = g.attributes.color;
        if (!ud.vc0 || ud.vc0.length !== a.array.length) { ud.vc0 = a.array.slice(); ud.vm = 's'; }
        if (ud.vm !== mode) {
          var st = a.itemSize;
          for (var i = 0; i < a.array.length; i += st) {
            var r = ud.vc0[i], gg = ud.vc0[i+1], bb = ud.vc0[i+2];
            if (hi) { r = s2l(r); gg = s2l(gg); bb = s2l(bb); }
            a.array[i] = r; a.array[i+1] = gg; a.array[i+2] = bb;
          }
          a.needsUpdate = true; ud.vm = mode;
        }
      }
    });
    // Schattenkamera an die schattenwerfenden Teile anpassen (nur wenn sich diese geändert haben)
    var key = keyParts.join(',');
    if (hi && key !== casterKey) { casterKey = key; fitShadow(); }
  }

  function fitShadow() {
    var box = new THREE.Box3(), tmp = new THREE.Box3();
    scene.traverse(function (o) { if (o.isMesh && o.castShadow && o.visible && o.geometry) { tmp.setFromObject(o); box.union(tmp); } });
    if (box.isEmpty()) box.set(new THREE.Vector3(-1500, -1500, 0), new THREE.Vector3(1500, 1500, 2000));
    var c = box.getCenter(new THREE.Vector3()), r = Math.max(800, box.getSize(new THREE.Vector3()).length() * 0.6);
    sun.target.position.set(c.x, c.y, 0);
    sun.position.copy(sun.target.position).addScaledVector(SUN_DIR, r * 3);
    var cam = sun.shadow.camera;
    cam.left = -r; cam.right = r; cam.top = r; cam.bottom = -r; cam.near = r * 0.5; cam.far = r * 6;
    cam.updateProjectionMatrix();
    sun.shadow.needsUpdate = true;
  }

  // ── Post-Processing (max) ────────────────────────────────────────
  // In r128 schreiben Materialien auch in Render-Targets mit Tonemapping und im Encoding des Targets.
  // Das Beauty-Target der SSAO bekommt deshalb sRGB: die ganze Kette arbeitet mit fertigen Farben,
  // Linien und Beschriftungen behalten ihre exakten Farben, FXAA läuft wie vorgesehen im sRGB-Raum.
  var comp = null, ssao = null, fxaa = null, compCam = null;
  function hasPost() { return THREE.EffectComposer && THREE.SSAOPass && THREE.ShaderPass && THREE.FXAAShader; }
  function buildComposer() {
    disposeComposer();
    var w = canvas.clientWidth || 1, h = canvas.clientHeight || 1, pr = renderer.getPixelRatio();
    comp = new THREE.EffectComposer(renderer);
    ssao = new THREE.SSAOPass(scene, activeCam, w * pr, h * pr);
    ssao.beautyRenderTarget.texture.encoding = THREE.sRGBEncoding;
    ssao.kernelRadius = 60;             // mm
    // Der Schattenboden bleibt im Normalen-Durchgang: ohne Fläche wertet die SSAO den leeren Hintergrund als verdeckt.
    ssao.kernelSize = 32;
    comp.addPass(ssao);
    fxaa = new THREE.ShaderPass(THREE.FXAAShader);
    comp.addPass(fxaa);
    compCam = activeCam;
    resize(w, h);
  }
  function disposeComposer() {
    if (!comp) return;
    try { ssao.dispose(); comp.renderTarget1.dispose(); comp.renderTarget2.dispose(); } catch (e) {}
    comp = ssao = fxaa = compCam = null;
  }
  function syncSSAO() {
    var cam = activeCam, u = ssao.ssaoMaterial.uniforms, persp = cam.isPerspectiveCamera ? 1 : 0;
    ssao.camera = cam;
    [ssao.ssaoMaterial, ssao.depthRenderMaterial].forEach(function (m) {
      if (!m) return;
      if (m.defines.PERSPECTIVE_CAMERA !== persp) { m.defines.PERSPECTIVE_CAMERA = persp; m.needsUpdate = true; }
      if (m.uniforms.cameraNear) { m.uniforms.cameraNear.value = cam.near; m.uniforms.cameraFar.value = cam.far; }
    });
    u.cameraProjectionMatrix.value.copy(cam.projectionMatrix);
    u.cameraInverseProjectionMatrix.value.copy(cam.projectionMatrixInverse);
    // Abstände als Anteil der Tiefe (cameraFar - cameraNear): 0,3 … 120 mm
    var range = Math.max(1, cam.far - cam.near);
    ssao.minDistance = 0.3 / range; ssao.maxDistance = 120 / range;
  }

  // ── Umschalten ───────────────────────────────────────────────────
  function apply() {
    var hi = quality !== 'standard';
    renderer.outputEncoding = hi ? THREE.sRGBEncoding : THREE.LinearEncoding;
    renderer.toneMapping = hi ? THREE.ACESFilmicToneMapping : THREE.NoToneMapping;
    renderer.toneMappingExposure = EXPOSURE;
    renderer.shadowMap.enabled = hi;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.shadowMap.needsUpdate = true;
    sun.castShadow = hi;
    if (amb) amb.intensity = hi ? LIGHT_HI.amb : LIGHT_STD.amb;
    sun.intensity = hi ? LIGHT_HI.sun : LIGHT_STD.sun;
    fill.intensity = hi ? LIGHT_HI.fill : LIGHT_STD.fill;
    if (!hi) { sun.position.copy(LIGHT_STD.sunPos); sun.target.position.set(0, 0, 0); }
    scene.environment = hi ? ensureEnv() : null;
    ground.visible = hi;
    casterKey = '';
    if (quality === 'max' && hasPost()) { if (!comp) buildComposer(); } else disposeComposer();
    // alle Materialien neu übersetzen lassen (Encoding/Tonemapping sind Shader-Defines)
    scene.traverse(function (o) { var ms = o.material ? (Array.isArray(o.material) ? o.material : [o.material]) : []; ms.forEach(function (m) { if (m) m.needsUpdate = true; }); });
    syncGround();
    document.querySelectorAll('[data-quality]').forEach(function (b) { b.classList.toggle('on', b.dataset.quality === quality); });
    var lbl = document.getElementById('quality-lbl'); if (lbl) lbl.textContent = 'Grafik ' + LBL[quality];
    requestRender(3);
  }
  function syncGround() {
    var t = document.documentElement.getAttribute('data-theme');
    ground.material.opacity = t === 'light' ? 0.18 : 0.32;
  }

  function render() {
    var hi = quality !== 'standard';
    prepare(hi);
    if (typeof renderMode !== 'undefined' && renderMode === 'wireframe') _applyRenderMode();
    if (comp && quality === 'max' && renderMode !== 'wireframe') {
      if (compCam !== activeCam) compCam = activeCam;
      syncSSAO();
      comp.render();
    } else {
      renderer.render(scene, activeCam);
    }
  }
  function resize(w, h) {
    if (!comp) return;
    var pr = renderer.getPixelRatio();
    comp.setPixelRatio(pr); comp.setSize(w, h);
    if (fxaa) fxaa.material.uniforms.resolution.value.set(1 / (w * pr), 1 / (h * pr));
  }
  function setQuality(q) {
    if (LEVELS.indexOf(q) < 0) return false;
    quality = q;
    try { localStorage.setItem(KEY, q); } catch (e) {}
    apply();
    if (typeof _robsimEmitState === 'function') _robsimEmitState();
    return true;
  }
  function cycle() { setQuality(LEVELS[(LEVELS.indexOf(quality) + 1) % LEVELS.length]); }

  window.RSGfx = {
    render: render, resize: resize, setQuality: setQuality, cycle: cycle,
    quality: function () { return quality; }, levels: LEVELS.slice(),
    themeChanged: function () { scene.userData.bgSet = null; syncGround(); requestRender(); }
  };
  apply();
})();
