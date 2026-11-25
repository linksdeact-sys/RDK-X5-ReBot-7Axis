import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';

const view = document.getElementById('urdf-view');
const canvas = document.getElementById('urdf-canvas');
const status = document.getElementById('urdf-status');
const gripperStatus = document.getElementById('urdf-gripper-state');
const reset = document.getElementById('urdf-reset');
const fallback = document.getElementById('arm-diagram');
let feedback;
window.addEventListener('rebot:feedback', ({detail}) => { feedback = detail; });

// Measured real-aperture endpoints, for display only. Match their relative
// travel in the URDF without assuming its nominal closed geometry is reached.
const OPEN_DEG = 49.3;
const INWARD_LIMIT_DEG = 241.3;
const ENDPOINT_TOLERANCE_DEG = 0.1;
const OPEN_APERTURE_MM = 60;
const LIMIT_APERTURE_MM = 8.5;
const FINGER_MAX_MM = 28.5;
const OPEN_REFERENCE = ' · 三维仅显示全开参考';

function gripperPose(frame) {
  const joint = frame?.joints?.find(j => j.id === 7);
  if (!frame?.usable || !joint?.online || !Number.isFinite(joint.angle)) {
    return null;
  }
  const atOpen = joint.angle <= OPEN_DEG + ENDPOINT_TOLERANCE_DEG;
  const atLimit = joint.angle >= INWARD_LIMIT_DEG - ENDPOINT_TOLERANCE_DEG;
  const progress = atOpen ? 0 : atLimit ? 1 :
    THREE.MathUtils.clamp((joint.angle - OPEN_DEG) / (INWARD_LIMIT_DEG - OPEN_DEG), 0, 1);
  const inwardTravelMm = progress * (OPEN_APERTURE_MM - LIMIT_APERTURE_MM);
  const fingerMm = FINGER_MAX_MM - inwardTravelMm / 2;
  const aperture = `${Number((OPEN_APERTURE_MM - inwardTravelMm).toFixed(2))} mm`;
  const percent = THREE.MathUtils.clamp(Math.round(progress * 100), 1, 99);
  const label = atOpen ? `最大张开（J7 ${OPEN_DEG}°） · 开口 ${aperture}` :
    atLimit ? `收拢限位（J7 ${INWARD_LIMIT_DEG}°） · 开口 ${aperture}` :
      `收拢行程 ${percent}% · 估算开口 ${aperture}`;
  return {fingerMm, label, aperture};
}

function setOrigin(object, {xyz, rpy}) {
  object.position.fromArray(xyz);
  // URDF fixed-axis roll, pitch, yaw: Rz(yaw) Ry(pitch) Rx(roll).
  object.rotation.set(...rpy, 'ZYX');
}

async function fetchModel(url, type) {
  const response = await fetch(url, {signal: AbortSignal.timeout(15000)});
  if (!response.ok) throw new Error('model HTTP ' + response.status);
  return response[type]();
}

async function start() {
  const model = await fetchModel('/api/urdf-view', 'json');
  const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#f2f7f4');
  const camera = new THREE.PerspectiveCamera(36, 1, .01, 10);
  camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, canvas);
  controls.minDistance = .2;
  controls.maxDistance = 4;
  controls.enablePan = false;
  const arm = new THREE.Group();
  scene.add(arm, new THREE.HemisphereLight(0xffffff, 0x9aa6a0, 2.3));
  const key = new THREE.DirectionalLight(0xffffff, 2.0);
  key.position.set(2, -3, 4);
  const fill = new THREE.DirectionalLight(0xd6eee7, 1.1);
  fill.position.set(-2, 2, 3);
  const grid = new THREE.GridHelper(1.7, 24, 0x90aaa0, 0xc9dad0);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -.002;
  scene.add(key, fill, grid);

  const links = new Map(model.links.map(({name}) => [name, new THREE.Group()]));
  const children = new Set();
  const actuators = new Map();
  for (const joint of model.joints) {
    const origin = new THREE.Group();
    const moving = new THREE.Group();
    setOrigin(origin, joint);
    links.get(joint.parent).add(origin);
    origin.add(moving);
    moving.add(links.get(joint.child));
    children.add(joint.child);
    actuators.set(joint.name, {moving, type: joint.type, axis: new THREE.Vector3(...joint.axis).normalize()});
  }
  const fingers = ['finger_left', 'finger_right'].map(name => actuators.get(name));
  const supportsGripper = model.name === 'ReBot_Arm_DM.urdf' &&
    fingers.every(finger => finger?.type === 'prismatic');
  const placeFingers = (fingerMm) => {
    // The two URDF joints share an axis; right mimics left with multiplier -1.
    const [left, right] = fingers;
    left.moving.position.copy(left.axis).multiplyScalar(fingerMm / 1000);
    right.moving.position.copy(right.axis).multiplyScalar(-fingerMm / 1000);
    view.dataset.fingerLeftMm = String(left.moving.position.dot(left.axis) * 1000);
    view.dataset.fingerRightMm = String(right.moving.position.dot(right.axis) * 1000);
  };
  if (supportsGripper) placeFingers(FINGER_MAX_MM);
  const roots = [...links].filter(([name]) => !children.has(name));
  if (roots.length !== 1) throw new Error('URDF visual tree invalid');
  arm.add(roots[0][1]);

  const loader = new STLLoader();
  const meshes = [];
  // Limit parallel reads so asset downloads don't crowd out state polling.
  for (let i = 0; i < model.meshes; i += 3) {
    meshes.push(...await Promise.all(Array.from({length: Math.min(3, model.meshes-i)},
      (_, j) => fetchModel('/urdf-view/mesh/' + (i+j), 'arrayBuffer').then(data => loader.parse(data)))));
  }
  for (const link of model.links) {
    for (const visual of link.visuals) {
      const color = new THREE.Color().setRGB(...visual.rgb, THREE.SRGBColorSpace);
      const mesh = new THREE.Mesh(meshes[visual.mesh], new THREE.MeshStandardMaterial({
        color, metalness: .1, roughness: .68,
      }));
      setOrigin(mesh, visual);
      mesh.scale.fromArray(visual.scale);
      links.get(link.name).add(mesh);
    }
  }
  const bounds = new THREE.Box3().setFromObject(arm);
  if (bounds.isEmpty()) throw new Error('URDF has no visual geometry');
  const sphere = bounds.getBoundingSphere(new THREE.Sphere());
  if (!Number.isFinite(sphere.radius) || sphere.radius <= 0) throw new Error('Invalid visual bounds');

  let raf = 0;
  const draw = () => {
    if (raf) return;
    raf = requestAnimationFrame(() => {
      raf = 0;
      renderer.render(scene, camera);
      view.dataset.frames = String((Number(view.dataset.frames) || 0) + 1);
    });
  };
  const resetView = () => {
    const direction = new THREE.Vector3(1, -1.7, .85).normalize();
    const distance = Math.max(.8, sphere.radius * 3.4 / Math.min(camera.aspect, 1));
    controls.target.copy(sphere.center);
    camera.position.copy(sphere.center).addScaledVector(direction, distance);
    controls.update();
    draw();
  };
  const resize = () => {
    const {width, height} = canvas.getBoundingClientRect();
    if (!width || !height) return;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
    draw();
  };

  let lastAngles = '';
  let lastGripper = null;
  const updatePose = () => {
    const angles = model.feedback.map(({id}) => {
      const joint = feedback?.joints?.find(j => j.id === id);
      return joint?.online && Number.isFinite(joint.angle) ? joint.angle : null;
    });
    const current = feedback?.usable && angles.every(angle => angle !== null);
    const pose = current ? angles.join(',') : '';
    if (current && pose !== lastAngles) {
      model.feedback.forEach(({joint, sign}, i) => {
        const {moving, axis} = actuators.get(joint);
        moving.quaternion.setFromAxisAngle(axis, THREE.MathUtils.degToRad(angles[i] * sign));
      });
      lastAngles = pose;
      draw();
    }
    if (supportsGripper) {
      const gripper = gripperPose(feedback);
      if (gripper) {
        if (gripper.fingerMm !== (lastGripper?.fingerMm ?? FINGER_MAX_MM)) {
          placeFingers(gripper.fingerMm);
          draw();
        }
        lastGripper = gripper;
      }
      const label = gripper ? `夹爪 · ${gripper.label}` : '夹爪 · 等待 J7 反馈' +
        (lastGripper ? ` · 保留上次显示（${lastGripper.aperture}）` : OPEN_REFERENCE);
      if (gripperStatus && gripperStatus.textContent !== label) gripperStatus.textContent = label;
    }
    const mode = current ? (feedback.simulate ? '仿真反馈' : '实测反馈') :
      (lastAngles ? '反馈中断 · 保留最后姿态' : '零位参考 · 等待反馈');
    if (status.textContent !== mode) status.textContent = mode;
    view.dataset.pose = current ? 'feedback' : lastAngles ? 'stale' : 'reference';
  };

  controls.addEventListener('change', draw);
  reset.addEventListener('click', resetView);
  canvas.addEventListener('webglcontextlost', () => {
    status.textContent = '3D 已中断 · 可刷新重试';
    view.hidden = true;
    fallback.removeAttribute('hidden');
    reset.hidden = true;
    view.dataset.pose = 'fallback';
  }, {once:true});
  canvas.addEventListener('webglcontextlost', () => window.removeEventListener('rebot:feedback', updatePose), {once:true});
  view.hidden = false;
  fallback.setAttribute('hidden', '');
  reset.hidden = false;
  new ResizeObserver(resize).observe(canvas);
  resize();
  resetView();
  updatePose();
  window.addEventListener('rebot:feedback', updatePose);
}

start().catch(error => {
  console.warn('reBot viewer:', error);
  status.textContent = '3D 暂不可用 · 显示示意图';
  view.dataset.pose = 'fallback';
});
