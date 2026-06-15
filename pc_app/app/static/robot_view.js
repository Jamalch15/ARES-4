import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

function jointPositions(anglesDeg, links) {
  const deg = Math.PI / 180;
  const base = anglesDeg[0] * deg;
  const shoulder = anglesDeg[1] * deg;
  const elbow = shoulder + anglesDeg[2] * deg;
  const wrist = elbow + anglesDeg[3] * deg;
  const lengths = [
    links.upper_arm_mm || 0,
    links.forearm_mm || 0,
    (links.wrist_mm || 0) + (links.tool_mm || 0),
  ];
  const pitches = [shoulder, elbow, wrist];
  const points = [{ x: 0, y: 0, z: links.base_height_mm || 0 }];
  let radial = 0;
  let z = links.base_height_mm || 0;

  for (let i = 0; i < lengths.length; i += 1) {
    radial += lengths[i] * Math.sin(pitches[i]);
    z += lengths[i] * Math.cos(pitches[i]);
    points.push({
      x: -radial * Math.sin(base),
      y: radial * Math.cos(base),
      z,
    });
  }

  return points;
}

function makeCylinderBetween(start, end, radius, material) {
  const startVec = start.clone();
  const endVec = end.clone();
  const direction = new THREE.Vector3().subVectors(endVec, startVec);
  const length = direction.length();
  const geometry = new THREE.CylinderGeometry(radius, radius, Math.max(length, 1), 16);
  const mesh = new THREE.Mesh(geometry, material);
  const midpoint = new THREE.Vector3().addVectors(startVec, endVec).multiplyScalar(0.5);
  mesh.position.copy(midpoint);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
  return mesh;
}

function robotToScene(point) {
  return new THREE.Vector3(point.x, point.z, -point.y);
}

function makeArmObjects(points, materials, radiusScale = 1) {
  const group = new THREE.Group();

  for (let i = 0; i < points.length - 1; i += 1) {
    group.add(
      makeCylinderBetween(
        points[i],
        points[i + 1],
        (i === points.length - 2 ? 9 : 12) * radiusScale,
        i === 1 ? materials.linkAlt : materials.link
      )
    );
  }

  points.forEach((point, index) => {
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry((index === points.length - 1 ? 12 : 16) * radiusScale, 24, 16),
      index === points.length - 1 ? materials.tool : materials.joint
    );
    sphere.position.set(point.x, point.y, point.z);
    group.add(sphere);
  });

  return group;
}

function disposeObject(object) {
  object.traverse((child) => {
    if (child.geometry) child.geometry.dispose();
  });
}

function clearGroup(group) {
  while (group.children.length) {
    const child = group.children[0];
    group.remove(child);
    disposeObject(child);
  }
}

function removeChildrenByKind(group, kind) {
  group.children
    .filter((child) => child.userData.kind === kind)
    .forEach((child) => {
      group.remove(child);
      disposeObject(child);
    });
}

function sameAngles(a, b) {
  return (
    Array.isArray(a) &&
    Array.isArray(b) &&
    a.length === b.length &&
    a.every((value, index) => Math.abs(Number(value) - Number(b[index])) < 0.0005)
  );
}

export class RobotView {
  constructor(container) {
    this.container = container;
    this.links = {};
    this.angles = [0, 0, 0, 0];
    this.previewVisible = true;
    this.pathVisible = true;
    this.framesVisible = true;
    this.previewAngles = null;
    this.lastRenderedAngles = null;
    this.lastConfigSignature = "";
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x171f2d);

    this.camera = new THREE.PerspectiveCamera(45, 1, 1, 2000);

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: false,
      powerPreference: "high-performance",
      preserveDrawingBuffer: true,
    });
    this.renderer.setClearColor(0x171f2d, 1);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);
    this.renderer.domElement.addEventListener("webglcontextlost", (event) => {
      event.preventDefault();
      this.container.dataset.webglStatus = "lost";
    });
    this.renderer.domElement.addEventListener("webglcontextrestored", () => {
      this.container.dataset.webglStatus = "ready";
      this.renderRobot();
      this.render();
    });
    this.container.dataset.webglStatus = "ready";

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.screenSpacePanning = true;
    this.controls.minDistance = 120;
    this.controls.maxDistance = 1200;
    this.controls.addEventListener("change", () => this.render());

    this.armGroup = new THREE.Group();
    this.previewGroup = new THREE.Group();
    this.overlayGroup = new THREE.Group();
    this.scene.add(this.armGroup);
    this.scene.add(this.previewGroup);
    this.scene.add(this.overlayGroup);

    const ambient = new THREE.AmbientLight(0xffffff, 0.8);
    const key = new THREE.DirectionalLight(0xffffff, 1.3);
    key.position.set(260, 520, 360);
    this.scene.add(ambient, key);

    this.grid = new THREE.GridHelper(720, 18, 0x2d3748, 0x202938);
    this.scene.add(this.grid);

    this.axes = new THREE.AxesHelper(180);
    this.scene.add(this.axes);

    this.materials = {
      base: new THREE.MeshStandardMaterial({ color: 0x0d1318, roughness: 0.55 }),
      link: new THREE.MeshStandardMaterial({ color: 0x0f6f69, roughness: 0.48 }),
      linkAlt: new THREE.MeshStandardMaterial({ color: 0xb98225, roughness: 0.5 }),
      joint: new THREE.MeshStandardMaterial({ color: 0xdce4ee, roughness: 0.4 }),
      tool: new THREE.MeshStandardMaterial({ color: 0xff6374, roughness: 0.42 }),
    };
    this.previewMaterials = {
      link: new THREE.MeshStandardMaterial({
        color: 0x6f96d1,
        roughness: 0.48,
        transparent: true,
        opacity: 0.34,
        depthWrite: false,
      }),
      linkAlt: new THREE.MeshStandardMaterial({
        color: 0xa58bd8,
        roughness: 0.48,
        transparent: true,
        opacity: 0.34,
        depthWrite: false,
      }),
      joint: new THREE.MeshStandardMaterial({
        color: 0xffffff,
        roughness: 0.4,
        transparent: true,
        opacity: 0.45,
        depthWrite: false,
      }),
      tool: new THREE.MeshStandardMaterial({
        color: 0x2f6bd1,
        roughness: 0.42,
        transparent: true,
        opacity: 0.55,
        depthWrite: false,
      }),
    };
    this.pathMaterial = new THREE.LineBasicMaterial({ color: 0x6aa7ff, linewidth: 2 });
    this.targetMaterial = new THREE.MeshStandardMaterial({
      color: 0xffd24a,
      emissive: 0x332300,
      roughness: 0.35,
    });

    this.resetCamera();
    window.addEventListener("resize", () => this.resize());
    this.resize();
    this.animate();
  }

  setConfig(config) {
    this.links = config.links_mm || {};
    this.lastConfigSignature = JSON.stringify(this.links);
    this.lastRenderedAngles = null;
    this.previewAngles = null;
    this.renderRobot();
  }

  setAngles(angles) {
    const normalizedAngles = angles.map(Number);
    if (sameAngles(normalizedAngles, this.lastRenderedAngles)) return;
    this.angles = normalizedAngles;
    this.container.dataset.currentAngles = this.angles.map((angle) => angle.toFixed(3)).join(",");
    this.renderRobot();
  }

  setPreviewAngles(angles) {
    if (!angles || angles.length !== 4) {
      clearGroup(this.previewGroup);
      this.previewAngles = null;
      delete this.container.dataset.previewAngles;
      this.render();
      return;
    }
    const normalizedAngles = angles.map(Number);
    if (sameAngles(normalizedAngles, this.previewAngles)) return;
    clearGroup(this.previewGroup);
    this.previewAngles = normalizedAngles;
    this.container.dataset.previewAngles = normalizedAngles.map((angle) => angle.toFixed(3)).join(",");
    const points = jointPositions(normalizedAngles, this.links).map(robotToScene);
    this.previewGroup.add(makeArmObjects(points, this.previewMaterials, 0.82));
    this.previewGroup.visible = this.previewVisible;
    this.render();
  }

  setTargetPoint(point) {
    removeChildrenByKind(this.overlayGroup, "target");
    if (!point) {
      delete this.container.dataset.targetPoint;
      this.render();
      return;
    }

    this.container.dataset.targetPoint = [
      Number(point.x_mm || 0).toFixed(3),
      Number(point.y_mm || 0).toFixed(3),
      Number(point.z_mm || 0).toFixed(3),
    ].join(",");
    const marker = new THREE.Mesh(new THREE.SphereGeometry(10, 24, 16), this.targetMaterial);
    marker.position.copy(
      robotToScene({
        x: Number(point.x_mm || 0),
        y: Number(point.y_mm || 0),
        z: Number(point.z_mm || 0),
      })
    );
    marker.userData.kind = "target";
    this.overlayGroup.add(marker);
    this.render();
  }

  setPathWaypoints(waypoints) {
    removeChildrenByKind(this.overlayGroup, "path");
    if (!waypoints || waypoints.length < 2) {
      delete this.container.dataset.pathWaypointCount;
      this.render();
      return;
    }

    this.container.dataset.pathWaypointCount = String(waypoints.length);
    const pathPoints = waypoints.map((angles) => {
      const joints = jointPositions(angles.map(Number), this.links);
      return robotToScene(joints[joints.length - 1]);
    });
    const geometry = new THREE.BufferGeometry().setFromPoints(pathPoints);
    const line = new THREE.Line(geometry, this.pathMaterial);
    line.userData.kind = "path";
    line.visible = this.pathVisible;
    this.overlayGroup.add(line);
    this.render();
  }

  setPreviewVisible(visible) {
    this.previewVisible = Boolean(visible);
    this.previewGroup.visible = this.previewVisible;
    this.render();
  }

  setPathVisible(visible) {
    this.pathVisible = Boolean(visible);
    this.overlayGroup.children
      .filter((child) => child.userData.kind === "path")
      .forEach((child) => {
        child.visible = this.pathVisible;
      });
    this.render();
  }

  setFramesVisible(visible) {
    this.framesVisible = Boolean(visible);
    this.grid.visible = this.framesVisible;
    this.axes.visible = this.framesVisible;
    this.render();
  }

  clearPreview() {
    clearGroup(this.previewGroup);
    clearGroup(this.overlayGroup);
    this.previewAngles = null;
    delete this.container.dataset.previewAngles;
    delete this.container.dataset.targetPoint;
    delete this.container.dataset.pathWaypointCount;
    this.render();
  }

  resize() {
    const width = Math.max(this.container.clientWidth, 1);
    const height = Math.max(this.container.clientHeight, 1);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
    this.render();
  }

  resetCamera() {
    const target = new THREE.Vector3(80, 120, -140);
    this.camera.position.set(620, 520, 660);
    this.camera.lookAt(target);
    if (this.controls) {
      this.controls.target.copy(target);
      this.controls.update();
    }
    this.render();
  }

  renderRobot() {
    clearGroup(this.armGroup);
    this.lastRenderedAngles = this.angles.slice();
    const points = jointPositions(this.angles, this.links).map(robotToScene);

    const baseHeight = this.links.base_height_mm || 80;
    const base = new THREE.Mesh(
      new THREE.CylinderGeometry(54, 66, 28, 32),
      this.materials.base
    );
    base.position.set(0, 14, 0);
    this.armGroup.add(base);

    const pedestal = makeCylinderBetween(
      robotToScene({ x: 0, y: 0, z: 28 }),
      robotToScene({ x: 0, y: 0, z: baseHeight }),
      22,
      this.materials.base
    );
    this.armGroup.add(pedestal);

    this.armGroup.add(makeArmObjects(points, this.materials));

    this.render();
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    this.controls.update();
    this.render();
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }
}
