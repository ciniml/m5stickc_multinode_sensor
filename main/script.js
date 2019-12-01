window.onload = function () {
    fetch('/config')
        .then(function(response) {
            return response.json();
        })
        .then(function(response) {
            let ssidElement = document.getElementsByName('ssid')[0];
            ssidElement.value = response.ssid;
            let passwordElement = document.getElementsByName('password')[0];
            passwordElement.value = response.password;
            let authmodeElement = document.getElementsByName('authmode')[0];
            authmodeElement.value = response.authmode;
            
        });
};

function applyConfig() {
    let params = {
        'ssid': document.getElementsByName('ssid')[0].value,
        'password': document.getElementsByName('password')[0].value,
        'authmode': document.getElementsByName('authmode')[0].value
    };

    let errorMessageElement = document.getElementById('error_message');
    fetch('/config', {"method": "POST", "body":JSON.stringify(params)} )
        .then(function(response) {
            if( response.ok ) {
                errorMessageElement.innerText = "Success";
            }
            else {
                errorMessageElement.innerText = "Error";
            }
        })
        .catch((e) => {
            errorMessageElement.innerText = e;
        });
}