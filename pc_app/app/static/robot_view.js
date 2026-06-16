import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

function jointPositions(anglesDeg, links) {
  const rows = Array.isArray(links.dh_rows) && links.dh_rows.length
    ? links.dh_rows
    : [
        { joint_index: 0, theta_offset_deg: 0, d_mm: links.base_height_mm || 0, a_mm: 0, alpha_deg: 90 },
        { joint_index: 1, theta_offset_deg: 90, d_mm: 0, a_mm: links.upper_arm_mm || 0, alpha_deg: 0 },
        { joint_index: 2, theta_offset_deg: 0, d_mm: 0, a_mm: links.forearm_mm || 0, alpha_deg: 0 },
        {
          joint_index: 3,
          theta_offset_deg: 0,
          d_mm: 0,
          a_mm: (links.wrist_mm || 0) + (links.tool_mm || 0),
          alpha_deg: 0,
        },
      ];
  let transform = identity4();
  const points = [robotPointFromDh(transform)];
  rows.forEach((row, fallbackIndex) => {
    const jointIndex = Number(row.joint_index ?? row.joint ?? fallbackIndex);
    const normalizedIndex = jointIndex >= 1 ? jointIndex - 1 : jointIndex;
    const theta =
      Number(anglesDeg[normalizedIndex] || 0) * Number(row.direction_sign ?? 1) +
      Number(row.zero_offset_deg || 0) +
      Number(row.theta_offset_deg || 0);
    transform = multiply4(
      transform,
      dhMatrix(theta, Number(row.d_mm || 0), Number(row.a_mm || 0), Number(row.alpha_deg || 0))
    );
    points.push(robotPointFromDh(transform));
  });
  return points;
}

function identity4() {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

function multiply4(a, b) {
  return a.map((row, rowIndex) =>
    row.map((_, columnIndex) =>
      [0, 1, 2, 3].reduce((sum, index) => sum + a[rowIndex][index] * b[index][columnIndex], 0)
    )
  );
}

function dhMatrix(thetaDeg, dMm, aMm, alphaDeg) {
  const theta = (thetaDeg * Math.PI) / 180;
  const alpha = (alphaDeg * Math.PI) / 180;
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  return [
    [ct, -st * ca, st * sa, aMm * ct],
    [st, ct * ca, -ct * sa, aMm * st],
    [0, sa, ca, dMm],
    [0, 0, 0, 1],
  ];
}

function robotPointFromDh(transform) {
  return { x: transform[1][3], y: -transform[0][3], z: transform[2][3] };
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
    this.objectGroup = new THREE.Group();
    this.scene.add(this.armGroup);
    this.scene.add(this.previewGroup);
    this.scene.add(this.overlayGroup);
    this.scene.add(this.objectGroup);

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
    this.objectMaterials = {
      red: new THREE.MeshStandardMaterial({ color: 0xff526d, emissive: 0x28040c, roughness: 0.4 }),
      green: new THREE.MeshStandardMaterial({ color: 0x53d18e, emissive: 0x052211, roughness: 0.4 }),
      blue: new THREE.MeshStandardMaterial({ color: 0x6aa7ff, emissive: 0x061629, roughness: 0.4 }),
      yellow: new THREE.MeshStandardMaterial({ color: 0xffd24a, emissive: 0x2d2203, roughness: 0.4 }),
      default: new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x121212, roughness: 0.4 }),
    };

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

  setObjectDetections(detections) {
    clearGroup(this.objectGroup);
    if (!Array.isArray(detections) || detections.length === 0) {
      delete this.container.dataset.objectMarkerCount;
      this.render();
      return;
    }

    let count = 0;
    detections.forEach((detection) => {
      const robot = detection.robot || {};
      if (robot.x_mm == null || robot.y_mm == null) return;
      const colorName = String(detection.color || "default").toLowerCase();
      const material = this.objectMaterials[colorName] || this.objectMaterials.default;
      const marker = new THREE.Mesh(new THREE.SphereGeometry(8, 24, 16), material);
      marker.position.copy(
        robotToScene({
          x: Number(robot.x_mm),
          y: Number(robot.y_mm),
          z: Number(robot.z_mm || 0),
        })
      );
      marker.userData.kind = "object";
      this.objectGroup.add(marker);
      count += 1;
    });
    this.container.dataset.objectMarkerCount = String(count);
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
