/*
 * HTML and Javascript code
 */
const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
  <style>
    button {
      min-width: 20ch;
      padding: 1rem;
      background-color: rgba(0, 0, 0, 1);
      border-style: none;
      color: white;
    }
    button:hover {
      cursor: pointer;
      background-color: rgba(0, 0, 0, 0.5);
    }
    body {
      display: flex;
      min-height: 100vh;
      justify-content: space-around;
      align-items: center;
      background-color: rgba(0, 0, 0, 0.8);
    }
    .text {
      margin-bottom: 2rem;
      text-align: center;
      font-weight: bold;
      padding-top: 1rem;
      padding-bottom: 1rem;
      color: white;
      box-sizing: border-box;
      min-height: 4rem;
      border: 1px solid red;
    }
  </style>
  <body>
    <div>
      <div class="text"><span id="state"> </span></div>
      <div>
        <button onclick="wallFollowing()">wallFollowing</button>
        <button onclick="beaconTracking()">beaconTracking</button>
        <button onclick="hit_full_speed()">full_speed</button>
      </div>
      <div>
        <button onclick="hit_reverse()">fall_back</button>
      </div>
      <div>
        <button onclick="slow_left()">ground_c_left</button>
        <button onclick="slow_right()">ground_c_right</button>
      </div>
      <div>
        <button onclick="ground_left()">ground_left</button>
        <button onclick="ground_right()">ground_right</button>
        <button onclick="hit_STOP_BUTTOM()">stop_button</button>
      </div>
    </div>
  </body>
  <script>
    function hit_full_speed() {
      var xhttp = new XMLHttpRequest();
      document.getElementById('state').innerHTML = 'FULLSPEED!!!!!!';
      xhttp.open('GET', 'hit_full_speed', true);
      xhttp.send();
    }

    function wallFollowing() {
      var xhttp = new XMLHttpRequest();
      document.getElementById('state').innerHTML = 'wall.';
      xhttp.open('GET', 'hit_wall', true);
      xhttp.send();
    }

    function beaconTracking() {
      var xhttp = new XMLHttpRequest();
      document.getElementById('state').innerHTML = 'beacon';
      xhttp.open('GET', 'hit_beacon', true);
      xhttp.send();
    }

    function hit_STOP_BUTTOM() {
      var xhttp = new XMLHttpRequest();
      document.getElementById('state').innerHTML = 'BREAK!';
      xhttp.open('GET', 'hit_STOP_BUTTOM', true);
      xhttp.send();
    }

    function hit_reverse() {
      var xhttp = new XMLHttpRequest();
      document.getElementById('state').innerHTML = 'fall back!!';
      xhttp.open('GET', 'hit_reverse', true);
      xhttp.send();
    }

    function slow_right() {
      var xhttp = new XMLHttpRequest();
      document.getElementById('state').innerHTML = 'slow_right';
      xhttp.open('GET', 'slow_right', true);
      xhttp.send();
    }

    function slow_left() {
      var xhttp = new XMLHttpRequest();
      document.getElementById('state').innerHTML = 'slow_left';
      xhttp.open('GET', 'slow_left', true);
      xhttp.send();
    }

    function ground_right() {
      var xhttp = new XMLHttpRequest();
      document.getElementById('state').innerHTML = 'ground_right';
      xhttp.open('GET', 'ground_right', true);
      xhttp.send();
    }

    function ground_left() {
      var xhttp = new XMLHttpRequest();
      document.getElementById('state').innerHTML = 'ground_left';
      xhttp.open('GET', 'ground_left', true);
      xhttp.send();
    }
  </script>
</html>
)===";
