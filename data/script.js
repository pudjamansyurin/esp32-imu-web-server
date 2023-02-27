/*
  Rui Santos
  Complete project details at 
  https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/

  Permission is hereby granted, free of charge, to any person obtaining a copy 
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in 
  all copies or substantial portions of the Software.
*/
const DPS_TO_RADS = (0.017453293);
const RADS_TO_DPS = (57.29577793);

let scene, camera, cube;

/* functions prototypes ------------------------------------------------------*/
function parentSize(elem) 
{
    return {
        w: elem.parentElement.clientWidth,
        h: elem.parentElement.clientHeight,
    }
}

function init3D()
{
    elem3d = document.getElementById("3Dcube");
    const { w, h } = parentSize(elem3d);

    // setup scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xffffff);

    // setup camera
    camera = new THREE.PerspectiveCamera(75, w / h, 0.1, 1000);
    camera.position.z = 5;

    // setup renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(w, h);
    elem3d.appendChild(renderer.domElement);

    // create cube
    const material = new THREE.MeshNormalMaterial({ wireframe: false }); 
    const geometry = new THREE.BoxGeometry(6, 1, 4);
    cube = new THREE.Mesh(geometry, material);
    scene.add(cube);

    // create axis 
    const axesHelper = new THREE.AxesHelper( 5 );
    scene.add(axesHelper);

    // combine object into scene
    renderer.render(scene, camera);
}

// Resize the 3D object when the browser window changes size
window.addEventListener('resize', function() {
    const { w, h } = parentSize(document.getElementById("3Dcube"));

    camera.aspect = w / h;
    //camera.aspect = window.innerWidth /  window.innerHeight;
    camera.updateProjectionMatrix();
    //renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setSize(w, h);
}, false);

// Create the 3D representation
init3D();

// Create events for the sensor readings
if (!!window.EventSource) 
{
    var source = new EventSource('/events');

    source.addEventListener('open', function(e) {
        console.log("Events Connected");
    }, false);

    source.addEventListener('error', function(e) {
        if (EventSource.OPEN != e.target.readyState) 
        {
            console.log("Events Disconnected");
        }
    }, false);

    source.addEventListener('readings', function(e) {
        var obj = JSON.parse(e.data);
        document.getElementById("gyroX").innerHTML = obj.gyroX;
        document.getElementById("gyroY").innerHTML = obj.gyroY;
        document.getElementById("gyroZ").innerHTML = obj.gyroZ;
        document.getElementById("acclX").innerHTML = obj.acclX;
        document.getElementById("acclY").innerHTML = obj.acclY;
        document.getElementById("acclZ").innerHTML = obj.acclZ;
        document.getElementById("magnX").innerHTML = obj.magnX;
        document.getElementById("magnY").innerHTML = obj.magnY;
        document.getElementById("magnZ").innerHTML = obj.magnZ;
        
        if (obj.tiltY)
        {
            document.getElementById("tiltY").innerHTML = obj.tiltY;
            document.getElementById("tiltR").innerHTML = obj.tiltR;
            document.getElementById("tiltP").innerHTML = obj.tiltP;
        }

        if (obj.quatX) {
            // change cube using quaternion
            const quaternion = new THREE.Quaternion(
                obj.quatX, obj.quatY, obj.quatZ, obj.quatW
            );
            cube.applyQuaternion(quaternion);
        } else {
            // change cube using euler angle
            cube.rotation.z = obj.tiltR * DPS_TO_RADS;
            cube.rotation.x = obj.tiltP * DPS_TO_RADS;
            cube.rotation.y = obj.tiltY * DPS_TO_RADS;
        }
        renderer.render(scene, camera);
    }, false);
}