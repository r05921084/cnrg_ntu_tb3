var chatObj = {
    host: location.host,
    socket: null,
    // Setting websocket
    init: function(){
        var url = "ws://" + chatObj.host + "/socket";
        chatObj.socket = new WebSocket(url);
        chatObj.socket.onmessage = function(event){
            chatObj.showMsg(event.data);
        },
        chatObj.socket.onclose = function(event){
            console.log("on close");
        },
        chatObj.socket.onerror = function(event){
            console.log("on error");
        }
    },
    // Send message to server
    sendMsg: function(message){
        // var msg_input = $("#msg_input");
        chatObj.socket.send(message);
        // msg_input.val("").select();
    },
    // Show the message
    showMsg: function(message){
        // $("#msg_box").append(message);
        var box = document.getElementById("chat_logs");
        var message_bubble = document.createElement('li');

        msg_json = parseJSON(message);
        if (msg_json){
            var message_info = document.createElement('li');
            message_info.className = 'message_info';
            message_info.innerHTML = msg_json.time + ' ' + msg_json.source;
            box.appendChild(message_info);

            message_bubble.className = 'message_bot';
            message_bubble.innerHTML = msg_json.text;
            box.appendChild(message_bubble);
        }
        else {
            message_bubble.className = 'message_bot';
            message_bubble.innerHTML = message;
            box.appendChild(message_bubble);
        }
        box.scrollTop = box.scrollHeight;
    }
};

function parseJSON(str){
    try {
        json = JSON.parse(str);
    } catch (e) {
        return false;
    }
    return json;
}



$(function(){
    var btn = $("#msg_btn");
    // var text_buffer = 0;

    btn.click(function(){
        text_area = document.getElementById("msg_input");
        if(text_area.value!=""){
            chatObj.sendMsg(text_area.value);
            var box = document.getElementById("chat_logs");
            var message_bubble = document.createElement('li');
            message_bubble.className = 'message_human';
            message_bubble.innerHTML = text_area.value;
            box.appendChild(message_bubble);
            text_area.value = "";
        }
        return false;
    });

    var input = document.getElementById("msg_input");
    input.addEventListener("keyup", function(event) {
    	event.preventDefault();
    	// Press enter will trigger the button
    	if (event.keyCode === 13) {
    		document.getElementById("msg_btn").click();
    	}
    });

    chatObj.init();
});
