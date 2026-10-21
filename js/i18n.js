'use strict';
// ═══════════════════════════════════════════════════════════
// i18n — Deutsch / English
// ═══════════════════════════════════════════════════════════
var I18N = {
  de: {
    // Toolbar
    play:'▶ Play', rev:'◀ Rev', stop:'⏸ Stop', step_f:'Step ▶', step_r:'◀ Step',
    robot3d:'Roboter 3D', tcptrace:'TCP Trace', bg:'◑ Hintergrund',
    settings:'⚙ Einst.', stlload:'↺ Anlage', clrtrace:'✕ Trace', reset:'⟳ Reset',
    axmap:'Achsenkarte', status_btn:'Status', grid_btn:'Grid',
    persp:'⊡ Perspektiv', ortho:'⊞ Ortho',
    // Panels
    kin_sec:'⚙ Kinematik', robo_set:'🎨 Robot Einstellungen',
    tcp_sec:'🔧 TCP · CH0', stl_sec:'📦 STL Szenenmodelle',
    pos_sec:'📍 Positionen', var_sec:'📦 Variablen',
    dout_sec:'🔴 Digital Outputs ($OUT)', din_sec:'🔵 Digital Inputs ($IN)',
    anout_sec:'📊 Analog ($ANOUT)',
    // Kinematik
    axlim:'Achsgrenzen [°]', linkoff:'Link-Offsets [mm] · Robot CS',
    applykin:'▶ Kinematik übernehmen', stlax:'STL Achsmodelle',
    new_kin:'+Neu', save:'💾 Speichern', load:'📂 Laden',
    // TCP
    tcppos:'Position [mm] relativ zum Flansch:',
    toolstl:'+ STL Werkzeug laden',
    overnahme:'▶ Übernehmen',
    // Settings panel
    sett_title:'⚙ Einstellungen', fontsizes:'Schriftgrössen',
    fz_editor:'Editor (Code)', fz_ui:'Toolbar / Buttons',
    fz_panel:'Parameter rechts', fz_status:'Statusleiste',
    fz_reset:'↺ Zurücksetzen', fz_save:'✓ Speichern',
    lang_label:'Sprache:', lang_de:'Deutsch', lang_en:'English',
    // Control panel
    ctrlpanel:'Steuerungspanel', phyaxes:'Physische Achsen',
    feedrate:'Vorschub [mm/min]',
    insert_stmt:'Satz einfügen (vor END):',
    ins_ptp:'+ PTP', ins_lin:'+ LIN',
    // Views
    iso:'Iso', top:'Top', bottom:'Bottom', front:'Front',
    back:'Back', left:'Links', right:'Rechts',
    // Robot settings
    linkcol:'Link-Farbe', jointcol:'Gelenk-Farbe',
    tcpcol:'TCP-Farbe', pathcol:'Path-Farbe',
    showtrace:'TCP Trace anzeigen', showws:'Workspace-Hülle',
    skeleton:'Skelett', axstl:'Achsen STL',
    toolstl2:'Werkzeug STL', pedstl:'Podest STL',
    // Empty states
    no_stl:'Keine STL Dateien geladen', no_pos:'Keine Positionen gefunden',
    no_pos2:'Keine Positionen', no_vars:'Keine Variablen',
    no_out:'Kein $OUT', no_in:'Kein $IN', no_anout:'Kein $ANOUT',
    // Hint
    hint:'LMB: Orbit / Pos. ziehen\u00a0|\u00a0RMB: Pan\u00a0|\u00a0Wheel: Zoom\u00a0|\u00a0Pos. klicken\u00a0=\u00a0auswählen',
    // Achsenkarte
    ampaxis:'📊 Achsenkarte · Redundanter Achsenoptimierer',
    amptab:'AxisA6 · Werkzeug-Drehachse',
    amp_collplan:'Kollisions Plan', amp_unreachable:'nicht Erreichbar',
    amp_limit:'Endschalter', amp_singular:'Singularitäten (A5≈0)',
    amp_valid:'Gültiger Bereich', amp_path:'Aktueller Weg',
    amp_optim:'Optimierung', amp_makeplan:'▶ Erstelle Plan',
    amp_apply:'✓ Weg übernehmen',
    // Parse button
    parse_btn:'▶ PARSE & LOAD',
    help_title:'? Hilfe · RobSimul V0.37',
    help_tab_start:'Überblick',
    help_tab_editor:'Editor',
    help_tab_robot:'Roboter',
    help_tab_keys:'Tastatur',
    // Confirm
    confirm_reset:'Alles zurücksetzen? Editor, Positionen und Roboterstellung werden auf Startwerte gesetzt.',
    confirm_new_kin:'Neue leere Kinematik erstellen? Aktuelle Einstellungen werden überschrieben.',
  },
  en: {
    // Toolbar
    play:'▶ Play', rev:'◀ Rev', stop:'⏸ Stop', step_f:'Step ▶', step_r:'◀ Step',
    robot3d:'Robot 3D', tcptrace:'TCP Trace', bg:'◑ Background',
    settings:'⚙ Settings', stlload:'↺ Anlage', clrtrace:'✕ Trace', reset:'⟳ Reset',
    axmap:'Axis Map', status_btn:'Status', grid_btn:'Grid',
    persp:'⊡ Perspective', ortho:'⊞ Ortho',
    // Panels
    kin_sec:'⚙ Kinematics', robo_set:'🎨 Robot Settings',
    tcp_sec:'🔧 TCP · CH0', stl_sec:'📦 STL Scene Models',
    pos_sec:'📍 Positions', var_sec:'📦 Variables',
    dout_sec:'🔴 Digital Outputs ($OUT)', din_sec:'🔵 Digital Inputs ($IN)',
    anout_sec:'📊 Analog ($ANOUT)',
    // Kinematics
    axlim:'Axis Limits [°]', linkoff:'Link Offsets [mm] · Robot CS',
    applykin:'▶ Apply Kinematics', stlax:'STL Axis Models',
    new_kin:'+New', save:'💾 Save', load:'📂 Load',
    // TCP
    tcppos:'Position [mm] relative to flange:',
    toolstl:'+ Load STL Tool',
    overnahme:'▶ Apply',
    // Settings panel
    sett_title:'⚙ Settings', fontsizes:'Font Sizes',
    fz_editor:'Editor (Code)', fz_ui:'Toolbar / Buttons',
    fz_panel:'Right Panel', fz_status:'Status Bar',
    fz_reset:'↺ Reset', fz_save:'✓ Save',
    lang_label:'Language:', lang_de:'Deutsch', lang_en:'English',
    // Control panel
    ctrlpanel:'Control Panel', phyaxes:'Physical Axes',
    feedrate:'Feed Rate [mm/min]',
    insert_stmt:'Insert statement (before END):',
    ins_ptp:'+ PTP', ins_lin:'+ LIN',
    // Views
    iso:'Iso', top:'Top', bottom:'Bottom', front:'Front',
    back:'Back', left:'Left', right:'Right',
    // Robot settings
    linkcol:'Link Color', jointcol:'Joint Color',
    tcpcol:'TCP Color', pathcol:'Path Color',
    showtrace:'Show TCP Trace', showws:'Workspace Envelope',
    skeleton:'Skeleton', axstl:'Axis STL',
    toolstl2:'Tool STL', pedstl:'Pedestal STL',
    // Empty states
    no_stl:'No STL files loaded', no_pos:'No positions found',
    no_pos2:'No positions', no_vars:'No variables',
    no_out:'No $OUT', no_in:'No $IN', no_anout:'No $ANOUT',
    // Hint
    hint:'LMB: Orbit / Drag Pos.\u00a0|\u00a0RMB: Pan\u00a0|\u00a0Wheel: Zoom\u00a0|\u00a0Click Pos.\u00a0=\u00a0select',
    // Achsenkarte
    ampaxis:'📊 Axis Map · Redundant Axis Optimizer',
    amptab:'AxisA6 · Tool Rotation Axis',
    amp_collplan:'Collision Plan', amp_unreachable:'Unreachable',
    amp_limit:'Limit Switch', amp_singular:'Singularities (A5≈0)',
    amp_valid:'Valid Range', amp_path:'Current Path',
    amp_optim:'Optimization', amp_makeplan:'▶ Create Plan',
    amp_apply:'✓ Apply Path',
    // Parse button
    parse_btn:'▶ PARSE & LOAD',
    help_title:'? Hilfe · RobSimul V0.37',
    help_tab_start:'Überblick',
    help_tab_editor:'Editor',
    help_tab_robot:'Roboter',
    help_tab_keys:'Tastatur',
    help_title:'? Help · RobSimul V0.37',
    help_tab_start:'Overview',
    help_tab_editor:'Editor',
    help_tab_robot:'Robot',
    help_tab_keys:'Keyboard',
    // Confirm
    confirm_reset:'Reset everything? Editor, positions and robot pose will be set to defaults.',
    confirm_new_kin:'Create new empty kinematics? Current settings will be overwritten.',
  }
};

var currentLang = 'de';

function detectLang() {
  try {
    var saved = localStorage.getItem('robsim_lang');
    if (saved === 'de' || saved === 'en') return saved;
    var bl = (navigator.language || navigator.userLanguage || 'de').toLowerCase();
    return bl.indexOf('de') === 0 ? 'de' : 'en';
  } catch(e) { return 'de'; }
}

function t(key) {
  var d = I18N[currentLang];
  return (d && d[key]) ? d[key] : (I18N.de[key] || key);
}

function applyLang() {
  document.querySelectorAll('[data-i18n]').forEach(function(el) {
    var key = el.getAttribute('data-i18n');
    var target = el.getAttribute('data-i18n-attr') || 'text';
    var val = t(key);
    if (target === 'text') { el.textContent = val; }
    else { el.setAttribute(target, val); }
  });
  // Update language buttons
  var btnDe = document.getElementById('lang-btn-de');
  var btnEn = document.getElementById('lang-btn-en');
  if (btnDe) btnDe.style.background = currentLang === 'de' ? 'var(--acc)' : '';
  if (btnDe) btnDe.style.color      = currentLang === 'de' ? '#fff' : '';
  if (btnEn) btnEn.style.background = currentLang === 'en' ? 'var(--acc)' : '';
  if (btnEn) btnEn.style.color      = currentLang === 'en' ? '#fff' : '';
}

function setLang(lang) {
  currentLang = lang;
  try { localStorage.setItem('robsim_lang', lang); } catch(e) {}
  applyLang();
}

// Init on load
currentLang = detectLang();
