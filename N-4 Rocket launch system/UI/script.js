// Handle Login
function handleLogin(event) {
    event.preventDefault();

    const username = document.getElementById('username').value.trim();
    const password = document.getElementById('password').value.trim();
    const errorMsg = document.getElementById('errorMsg');

    if (username === 'NAKUJA' && password === '12345') {
        // Hide login and show dashboard
        document.getElementById('loginPage').style.display = 'none';
        document.getElementById('dashboard').style.display = 'block';

        // Initialize gauges after successful login
        initGauges();

        // Start WebSocket connection after successful login
        setupWebSocket();

    } else {
        errorMsg.textContent = 'Incorrect username or password';
    }
}

// Initialize Gauges
function initGauges() {
    const gaugeTemp = new LinearGauge({
        renderTo: 'gauge-temperature',
        width: 120,
        height: 400,
        units: "Temperature (°C)",
        minValue: 0,
        maxValue: 125,
        majorTicks: Array.from({ length: 26 }, (_, i) => (i * 5).toString()),
        highlights: [{ from: 80, to: 125, color: "rgba(200, 50, 50, .75)" }],
        animationDuration: 1500,
    }).draw();

    const gaugePres = new RadialGauge({
        renderTo: 'gauge-Pressure',
        width: 300,
        height: 300,
        units: "Pressure (MPa)",
        minValue: 0,
        maxValue: 10,
        majorTicks: ["0", "2", "4", "6", "8", "10"],
        highlights: [{ from: 6, to: 10, color: "#FF4500" }],
        animationDuration: 1500,
    }).draw();

    const gaugeThrust = new RadialGauge({
        renderTo: 'gauge-Thrust',
        width: 300, // Adjust based on preferred dimensions
        height: 300,
        units: "Thrust (Kgf)",
        minValue: 0,
        maxValue: 800,
        majorTicks: ["0", "200", "400", "600", "800"],
        highlights: [{ from: 600, to: 800, color: "rgba(255, 0, 0, .75)" }],
        animationDuration: 1500,
    }).draw();
    
    // Store gauges for later updates
    window.gauges = { gaugeTemp, gaugePres, gaugeThrust };

    // Simulate initial values
    gaugeTemp.value = 100; // Example initial temperature
    gaugePres.value = 6;  // Example initial pressure
    gaugeThrust.value = 600; // Example initial thrust value in kgf

}

    // WebSocket setup to listen for data from the ESP32
function setupWebSocket() {
    const socket = new WebSocket('ws://' + window.location.hostname + ':81'); // Adjust port if needed
    //const socket = new WebSocket('ws://' + window.location.hostname + ':8080'); // Adjust port if needed

    socket.onopen = function() {
        console.log('WebSocket connection established');
    };

    socket.onmessage = function(event) {
        const data = JSON.parse(event.data); // Assuming the data is sent as JSON
        collectData(data);   // Call collectData() with the incoming data from ESP32
        updateGauges(data);  // Update gauges with incoming data
        
    };

    socket.onerror = function(error) {
        console.error('WebSocket Error: ', error);
    };

    socket.onclose = function() {
        console.log('WebSocket connection closed');
    };
}

// Update gauges with new data from ESP32
function updateGauges(data) {
    if (window.gauges) {
        const { gaugeTemp, gaugePres, gaugeThrust } = window.gauges;

        if (data.temperature !== undefined) {
            gaugeTemp.value = data.temperature;
        }
        if (data.pressure !== undefined) {
            gaugePres.value = data.pressure;
        }
        if (data.thrust !== undefined) {
            gaugeThrust.value = data.thrust;
        }
    }
}

// Start WebSocket connection after login
function startWebSocketConnection() {
    setupWebSocket();
}

// Call the function when login is successful
// Call the function when login is successful
document.getElementById('loginForm').addEventListener('submit', function(event) {
    handleLogin(event);

//document.getElementById('loginForm').addEventListener('submit', function(event) {
    //handleLogin(event);
    //startWebSocketConnection(); // Start the WebSocket connection after login
});


// Variable to store collected data
let recordedData = [];
let recording = false;
let dataCollectionInterval = null;

// Function to start recording
function startRecording() {
    if (!recording) {
        recording = true;
        recordedData = []; // Reset data when starting new recording
        document.getElementById('startRecordingBtn').disabled = true;
        document.getElementById('stopRecordingBtn').disabled = false;
        console.log("Recording Started...");
        
        // Start collecting data every 1 second (adjust interval as needed)
        dataCollectionInterval = setInterval(collectData, 1000);
    }
}

// Function to stop recording
function stopRecording() {
    if (recording) {
        recording = false;
        document.getElementById('startRecordingBtn').disabled = false;
        document.getElementById('stopRecordingBtn').disabled = true;
        console.log("Recording Stopped.");
        
        // Stop collecting data
        clearInterval(dataCollectionInterval);
    }
}

    // Function to collect data from the ESP32 and store it
function collectData(espData) {
    if (recording) {
        const data = {
            timestamp: new Date().toISOString(), // creates a timestamp
            temperature: espData.temperature,   // Real temperature value from ESP32
            pressure: espData.pressure,         // Real pressure value from ESP32
            thrust: espData.thrust              // Real thrust value from ESP32
        };
        recordedData.push(data);
    }
}


// Function to simulate collecting data
//function collectData() {
    //if (recording) {
       // const data = {
           // timestamp: new Date().toISOString(), //creates a timestamp
            //temperature: gaugeTemp.value,  // Gauge values
            //pressure: gaugePres.value,
           //thrust: gaugeThrust.value
        //};
       // recordedData.push(data);
   // }
//}

// Function to download data as CSV
function downloadData() {
    if (recordedData.length === 0) {
        alert("No data to download.");
        return;
    }

    // Prepare CSV
    const headers = ['Timestamp', 'Temperature (°C)', 'Pressure (MPa)', 'Thrust (Kgf)'];
    const rows = recordedData.map(item => [
        item.timestamp,
        item.temperature,
        item.pressure,
        item.thrust
    ]);
    const csvContent = [headers, ...rows].map(e => e.join(",")).join("\n");

    // Create a Blob and trigger download
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const link = document.createElement('a');
    const url = URL.createObjectURL(blob);
    link.setAttribute('href', url);
    link.setAttribute('download', 'sensor_data.csv');
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}
