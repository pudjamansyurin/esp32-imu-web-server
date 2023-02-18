/*
  Rui Santos
  Complete project details at 
  https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/

  Permission is hereby granted, free of charge, to any person obtaining a copy 
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in 
  all copies or substantial portions of the Software.
*/

let scene, camera, cube;

/* functions prototypes ------------------------------------------------------*/
function rad2deg (rad, precision = 2) 
{
    return (parseFloat(rad) * (180 / Math.PI)).toFixed(precision);
};

function resetPosition(elem)
{
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/"+elem.id, true);
    xhr.send();
  }

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

    // create cube
    const material = new THREE.MeshFaceMaterial([
        new THREE.MeshBasicMaterial({color:0x03045e}),
        new THREE.MeshBasicMaterial({color:0x023e8a}),
        new THREE.MeshBasicMaterial({color:0x0077b6}),
        new THREE.MeshBasicMaterial({color:0x03045e}),
        new THREE.MeshBasicMaterial({color:0x023e8a}),
        new THREE.MeshBasicMaterial({color:0x0077b6}),
    ]);
    const geometry = new THREE.BoxGeometry(6, 1, 4);
    cube = new THREE.Mesh(geometry, material);

    // create axis 
    const axesHelper = new THREE.AxesHelper( 5 );

    // setup scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xffffff);
    
    // combine object into scene
    scene.add(axesHelper);
    scene.add(cube);

    // setup camera
    camera = new THREE.PerspectiveCamera(75, w / h, 0.1, 1000);
    camera.position.z = 5;

    // setup renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(w, h);
    renderer.render(scene, camera);

    elem3d.appendChild(renderer.domElement);
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
        document.getElementById("temp").innerHTML  = obj.temp;
        document.getElementById("gyroX").innerHTML = obj.gyroX;
        document.getElementById("gyroY").innerHTML = obj.gyroY;
        document.getElementById("gyroZ").innerHTML = obj.gyroZ;
        document.getElementById("acclX").innerHTML = obj.acclX;
        document.getElementById("acclY").innerHTML = obj.acclY;
        document.getElementById("acclZ").innerHTML = obj.acclZ;
        document.getElementById("tiltYaw").innerHTML   = rad2deg(obj.tiltYaw);
        document.getElementById("tiltRoll").innerHTML  = rad2deg(obj.tiltRoll);
        document.getElementById("tiltPitch").innerHTML = rad2deg(obj.tiltPitch);

        // Change cube rotation after receiving the readinds
        cube.rotation.z = obj.tiltRoll;
        cube.rotation.x = obj.tiltPitch;
        cube.rotation.y = obj.tiltYaw;
        renderer.render(scene, camera);
    }, false);
}