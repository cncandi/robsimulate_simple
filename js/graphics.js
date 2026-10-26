// ═══════════════════════════════════════════════════════════════════
// RobSimul – Grafikqualität (gemeinsame Datei für RobSimul und RobSimul Simple, three.js r128)
//   standard : bisherige Darstellung (Phong, ohne Schatten) – schnell, für schwache Rechner
//   high     : PBR (MeshPhysicalMaterial), Studio-Umgebung (RoomEnvironment), weiche Schatten,
//              ACES-Tonemapping, sRGB-Ausgabe
//   max      : wie high + Ambient Occlusion (SSAO) + FXAA (Post-Processing)
// In allen Stufen: STL-Normalen mit Knickwinkel geglättet (Rundungen glatt, Kanten scharf).
//
// Erwartet globale Bindungen aus simulator.js: renderer, scene, sun, fill, activeCam, canvas, requestRender;
// optional: orbitTarget, renderMode/_applyRenderMode, STLLoader, _robsimEmitState.
// Laden nach simulator.js:  <script src="js/graphics.js" data-key="robsimul.quality"></script>
// Umschalten: RSGfx.setQuality('standard'|'high'|'max'), RSGfx.cycle(), URL ?quality=…
// Bedienelemente: [data-quality="…"] bekommen .on, [data-gfx-label] zeigt „Grafik …“.
// ═══════════════════════════════════════════════════════════════════
(function () {
  'use strict';
  var LEVELS = ['standard', 'high', 'max'], LBL = { standard: 'Standard', high: 'Hoch', max: 'Maximal' };
  var me = document.currentScript;
  var KEY = (me && me.dataset.key) || 'robsim.quality';
  var quality = 'high';
  try {
    var qp = new URLSearchParams(location.search).get('quality') || localStorage.getItem(KEY);
    if (LEVELS.indexOf(qp) >= 0) quality = qp;
  } catch (e) {}

  // ── Glatte STL-Normalen ──────────────────────────────────────────
  // Normale je Ecke aus allen angrenzenden Dreiecken mitteln, aber nur über Kanten unter dem Knickwinkel.
  // (computeVertexNormals allein wirkt bei STL nicht: jedes Dreieck hat eigene Eckpunkte.)
  function smoothNormals(geo, creaseDeg) {
    var pos = geo.attributes.position;
    if (!pos) return geo;
    if (geo.index) { THREE.BufferGeometry.prototype.computeVertexNormals.call(geo); return geo; }
    var p = pos.array, nV = pos.count, nT = Math.floor(nV / 3), cosC = Math.cos((creaseDeg || 32) * Math.PI / 180);
    var fn = new Float32Array(nT * 3), fu = new Float32Array(nT * 3);
    for (var t = 0; t < nT; t++) {
      var i = t * 9, ax = p[i+3]-p[i], ay = p[i+4]-p[i+1], az = p[i+5]-p[i+2], bx = p[i+6]-p[i], by = p[i+7]-p[i+1], bz = p[i+8]-p[i+2];
      var nx = ay*bz-az*by, ny = az*bx-ax*bz, nz = ax*by-ay*bx, l = Math.hypot(nx, ny, nz) || 1;
      fn[t*3] = nx; fn[t*3+1] = ny; fn[t*3+2] = nz; fu[t*3] = nx/l; fu[t*3+1] = ny/l; fu[t*3+2] = nz/l;
    }
    var map = new Map(), key = new Array(nV), v;
    for (v = 0; v < nV; v++) {
      var k = Math.round(p[v*3]*500) + '_' + Math.round(p[v*3+1]*500) + '_' + Math.round(p[v*3+2]*500);
      key[v] = k; var l2 = map.get(k); if (l2) l2.push(v / 3 | 0); else map.set(k, [v / 3 | 0]);
    }
    var out = new Float32Array(nV * 3);
    for (v = 0; v < nV; v++) {
      var t0 = v / 3 | 0, list = map.get(key[v]), sx = 0, sy = 0, sz = 0;
      for (var j = 0; j < list.length; j++) {
        var t1 = list[j];
        if (fu[t0*3]*fu[t1*3] + fu[t0*3+1]*fu[t1*3+1] + fu[t0*3+2]*fu[t1*3+2] >= cosC) { sx += fn[t1*3]; sy += fn[t1*3+1]; sz += fn[t1*3+2]; }
      }
      var ln = Math.hypot(sx, sy, sz) || 1; out[v*3] = sx/ln; out[v*3+1] = sy/ln; out[v*3+2] = sz/ln;
    }
    geo.setAttribute('normal', new THREE.BufferAttribute(out, 3));
    geo.userData.smoothed = true;
    return geo;
  }
  // STL-Lader: geglättete Normalen; ein späteres geo.computeVertexNormals() glättet ebenso (statt flach)
  if (typeof STLLoader !== 'undefined' && STLLoader.prototype && STLLoader.prototype.parse && !STLLoader.prototype._gfx) {
    var origParse = STLLoader.prototype.parse;
    STLLoader.prototype.parse = function (buf) {
      var g = origParse.call(this, buf);
      smoothNormals(g, 32);
      g.computeVertexNormals = function () { if (!this.userData.smoothed || this.attributes.position.version !== this.userData.posVer) smoothNormals(this, 32); this.userData.posVer = this.attributes.position.version; };
      g.userData.posVer = g.attributes.position.version;
      return g;
    };
    STLLoader.prototype._gfx = true;
  }

  // ── Lichter der Szene ────────────────────────────────────────────
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

  // Boden, der nur Schatten empfängt
  var ground = new THREE.Mesh(new THREE.PlaneGeometry(60000, 60000),
    new THREE.ShadowMaterial({ opacity: 0.3, transparent: true, depthWrite: false }));
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
  // Phong-Materialien (Roboterglieder, Podest, Werkzeug, Bauteile) bekommen in high/max ein PBR-Gegenstück.
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
      transparent: m.transparent, opacity: m.opacity, side: m.side,
      depthWrite: m.depthWrite, depthTest: m.depthTest, wireframe: m.wireframe
    });
    p.userData.fromStd = true;
    return p;
  }
  function s2l(v) { return v <= 0.04045 ? v * 0.0773993808 : Math.pow(v * 0.9478672986 + 0.0521327014, 2.4); }

  var casterKey = '';
  function prepare(hi) {
    var keyParts = [], mode = hi ? 'h' : 's';
    scene.traverse(function (o) {
      if (o.userData.gfx) return;
      // Material tauschen
      if (o.isMesh && o.material && !Array.isArray(o.material)) {
        // Code kann das Material eines Meshes jederzeit ersetzen: dann neu zuordnen
        if (o.material.isMeshPhongMaterial && o.userData.matStd !== o.material && o.material !== o.userData.matPhys) { o.userData.matStd = o.material; o.userData.matPhys = null; }
        var ms = o.userData.matStd;
        if (ms && (o.material === ms || o.material === o.userData.matPhys)) {
          if (hi && !o.userData.matPhys) o.userData.matPhys = physFrom(ms);
          var mp = o.userData.matPhys;
          if (mp) { mp.opacity = ms.opacity; mp.transparent = ms.transparent; mp.visible = ms.visible; mp.side = ms.side; }
          o.material = hi ? mp : ms;
          if (o.userData.castShadow0 === undefined) o.userData.castShadow0 = o.castShadow;
          o.castShadow = hi && !(ms.transparent && ms.opacity < 0.5); o.receiveShadow = hi;
          if (hi && o.visible) keyParts.push(o.id);
        }
      }
      // Farben: standard = Original; high/max = linear gerechnet, Ausgabe sRGB.
      // Ändert der Code eine Farbe (z. B. Theme, Hervorhebung), wird die neue Farbe übernommen.
      var mats = o.material ? (Array.isArray(o.material) ? o.material : [o.material]) : [];
      mats.forEach(function (m) {
        if (!m || !m.color) return;
        var u = m.userData;
        if (!u.c0 || (u.cw && !m.color.equals(u.cw))) { u.c0 = m.color.clone(); u.cm = null; }
        if (m.emissive && (!u.e0 || (u.ew && !m.emissive.equals(u.ew)))) { u.e0 = m.emissive.clone(); u.cm = null; }
        if (u.cm === mode) return;
        var pbr = m.isMeshPhysicalMaterial || m.isMeshStandardMaterial || m.isMeshPhongMaterial || m.isMeshLambertMaterial;
        m.color.copy(u.c0); if (hi) m.color.convertSRGBToLinear();
        if (u.e0) { m.emissive.copy(u.e0); if (hi) m.emissive.convertSRGBToLinear(); u.ew = m.emissive.clone(); }
        u.cw = m.color.clone();
        // Linien, Punkte, Beschriftungen, Marker: exakte Farben, kein Tonemapping
        if (!pbr) m.toneMapped = false;
        if (m.map) { m.map.encoding = hi ? THREE.sRGBEncoding : THREE.LinearEncoding; m.map.needsUpdate = true; }
        u.cm = mode; m.needsUpdate = true;
      });
      var g = o.geometry;
      if (g && g.attributes && g.attributes.color) {
        var ud = g.userData, a = g.attributes.color;
        if (!ud.vc0 || ud.vc0.length !== a.array.length || (ud.vver !== undefined && a.version !== ud.vver)) { ud.vc0 = a.array.slice(); ud.vm = 's'; }
        if (ud.vm !== mode) {
          var st = a.itemSize;
          for (var i = 0; i < a.array.length; i += st) {
            var r = ud.vc0[i], gg = ud.vc0[i+1], bb = ud.vc0[i+2];
            if (hi) { r = s2l(r); gg = s2l(gg); bb = s2l(bb); }
            a.array[i] = r; a.array[i+1] = gg; a.array[i+2] = bb;
          }
          a.needsUpdate = true; ud.vm = mode;
        }
        ud.vver = a.version;
      }
    });
    // Schattenkamera an die schattenwerfenden Teile anpassen (nur wenn sich diese geändert haben)
    var key = keyParts.join(',');
    if (hi && key !== casterKey) { casterKey = key; fitShadow(); }
  }

  function fitShadow() {
    var box = new THREE.Box3(), tmp = new THREE.Box3();
    scene.traverse(function (o) { if (o.isMesh && o.castShadow && o.visible && o.geometry) { tmp.setFromObject(o); if (!tmp.isEmpty() && isFinite(tmp.min.x)) box.union(tmp); } });
    if (box.isEmpty()) box.set(new THREE.Vector3(-1500, -1500, 0), new THREE.Vector3(1500, 1500, 2000));
    var c = box.getCenter(new THREE.Vector3()), r = Math.min(20000, Math.max(800, box.getSize(new THREE.Vector3()).length() * 0.6));
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
  var comp = null, ssao = null, fxaa = null;
  function hasPost() { return THREE.EffectComposer && THREE.SSAOPass && THREE.ShaderPass && THREE.FXAAShader; }
  function buildComposer() {
    disposeComposer();
    var w = canvas.clientWidth || 1, h = canvas.clientHeight || 1, pr = renderer.getPixelRatio();
    comp = new THREE.EffectComposer(renderer);
    ssao = new THREE.SSAOPass(scene, activeCam, w * pr, h * pr);
    ssao.beautyRenderTarget.texture.encoding = THREE.sRGBEncoding;
    ssao.kernelRadius = 60;             // mm
    ssao.kernelSize = 32;
    // Der Schattenboden bleibt im Normalen-Durchgang: ohne Fläche wertet die SSAO den leeren Hintergrund als verdeckt.
    comp.addPass(ssao);
    fxaa = new THREE.ShaderPass(THREE.FXAAShader);
    comp.addPass(fxaa);
    resize(w, h);
  }
  function disposeComposer() {
    if (!comp) return;
    try { ssao.dispose(); comp.renderTarget1.dispose(); comp.renderTarget2.dispose(); } catch (e) {}
    comp = ssao = fxaa = null;
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
  // Tiefenbereich an den Abstand zum Drehpunkt anpassen (genauere Tiefe für Schatten und AO)
  var tgt = new THREE.Vector3();
  function fitDepth(cam) {
    if (typeof orbitTarget === 'undefined' || !orbitTarget) return;
    var d = cam.position.distanceTo(orbitTarget), n, f;
    if (cam.isPerspectiveCamera) { n = Math.max(2, d * 0.02); f = d + 30000; }
    else { n = d - 30000; f = d + 30000; }
    if (cam.near !== n || cam.far !== f) { cam.near = n; cam.far = f; cam.updateProjectionMatrix(); }
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
    document.querySelectorAll('[data-quality]').forEach(function (b) { b.classList.toggle('on', b.dataset.quality === quality); });
    document.querySelectorAll('[data-gfx-label]').forEach(function (el) { el.textContent = 'Grafik ' + LBL[quality]; });
    requestRender(3);
  }
  // Schattenstärke nach Hintergrundhelligkeit (helle Themes: zarter)
  function syncGround() {
    var b = scene.background && scene.background.isColor ? scene.background : null;
    var lum = b ? 0.2126 * b.r + 0.7152 * b.g + 0.0722 * b.b : 0;
    ground.material.opacity = lum > 0.5 ? 0.18 : 0.32;
  }

  function render() {
    var hi = quality !== 'standard';
    prepare(hi);
    syncGround();
    var wire = typeof renderMode !== 'undefined' && renderMode === 'wireframe';
    if (wire && typeof _applyRenderMode === 'function') _applyRenderMode();
    if (hi) fitDepth(activeCam);
    if (comp && quality === 'max' && !wire) { syncSSAO(); comp.render(); }
    else renderer.render(scene, activeCam);
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
    render: render, resize: resize, setQuality: setQuality, cycle: cycle, smoothNormals: smoothNormals,
    quality: function () { return quality; }, levels: LEVELS.slice(),
    themeChanged: function () { requestRender(); }
  };
  apply();
})();
