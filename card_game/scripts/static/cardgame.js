
var imgQues = new Image();
imgQues.src = 'static/images/ques.png';
var cards = new Array();
var server_msg;
var card_num;
var card_state = new Array();
var pre_card_idx = -1;
webSocket_op();

function loadImg(card_names){

	for (var i=0; i<card_names.length; i++) {
		cards[i] = new Image();
		cards[i].src = 'static/images/' + card_names[i];
	}
}


function loadImg1(){
	// var imgQues = new Image();
	// imgQues.src = "pic/ques.png";
	
	var card_names = ["apple.png", "banana.png", "cat.png", "dog.png"];

	for (var i=0; i<card_names.length; i++) {
		cards[i] = new Image();
		cards[i].src = "static/images/" + card_names[i];
	} 

}


window.onload = function() {
	// TODO : https://stackoverflow.com/questions/7171483/simple-way-to-get-element-by-id-within-a-div-tag
};


function initCardstate() {
	// Initial card state true = front 
	// 					  false = back
	for(i=0; i<card_num; i++){
		card_state[i] = false;
	}
}


function startClick() {

	var card_names = ["apple.png", "banana.png", "cat.png", "dog.png"]
	var card_index = ["card0", "card1", "card2", "card3"]

	for(i=0;i<4;i++){
    	var canvas = document.getElementById(card_index[i]);
    	var ctx = canvas.getContext("2d");
    	ctx.clearRect(0, 0, canvas.width, canvas.height);
    	ctx.drawImage(cards[i],0,0);
    }
}


function guessClick() {

	var card_index = ["card0", "card1", "card2", "card3"]
	for(i=0;i<4;i++){
    	var canvas = document.getElementById(card_index[i]);
    	var ctx = canvas.getContext("2d");
    	ctx.clearRect(0, 0, canvas.width, canvas.height);
    	ctx.drawImage(imgQues,50,70);
    }

}


function messageDecode(msg) {
	msg_arr = msg.split(',');
	act = msg_arr[0];
	content = msg_arr.slice(1,);

	if(act=='start'){
		start();
		doSend('>'+act);
	}
	else if(act=='fold'){
		fold();
		doSend('>'+act);
	}
	else if(act=='load'){
		loadImg(content);
		doSend('>'+act);
	}
	else if(act=='ask'){
		// alert(content);
		asking(content);
		doSend('>'+act);
	}
	else if(act=='test'){
		// alert(content);
		loadImg1();
		start();
		doSend('>'+act);
	}
	else if(act=='talk'){
		document.getElementById("textBox").innerHTML = content;
		doSend('>'+act);
	}
	else if(act=='init'){
		init(content);
		doSend('>'+act);
	}
	else if(act=='turn'){
		turn(content);
		doSend('>'+act);
	}
	else{
		doSend('> ERROR: No such instruction !');
	}

}

// ?????????
function turn(card_id){
	card_state[card_id] = true;
	var canvas = document.getElementById('card'+card_id);
	var ctx=canvas.getContext("2d");
	canvas.width=450;
	canvas.height=600;
	ctx.drawImage(cards[card_id], 0, 0, canvas.width, canvas.height);
}


function init(content){

	card_num = content;
	// var card_board = document.createElement("div");

	// Clear div element
	document.getElementById("content").innerHTML = "";

	var card_board = document.getElementById("content");

	for(i=0; i<card_num; i++){
		var canvas = document.createElement('canvas');
		canvas.id = "card"+i;
		if(card_num<=4){
			canvas.width = 300;
			canvas.height = 400;
		}
		else{

		}
		canvas.style.border = "1px solid #d3d3d3";

		card_board.appendChild(canvas);
		var ctx = canvas.getContext("2d");
		ctx.drawImage(imgQues,x=0,y=0, width=canvas.width, height=canvas.height);
	}





	// card_num = int(content);
	// for(i=0;i<card_num;i++){
 //    	var canvas = document.getElementById("card"+i);
 //    	canvas.width = 300;
 //    	canvas.height = 400;
 //    	var ctx = canvas.getContext("2d");
 //    	ctx.drawImage(imgQues,x=0,y=0, width=canvas.width, height=canvas.height);
 //    }
 //    document.getElementById("textBox").innerHTML = "Hello";

 //    for(i=0; i<card_num; i++){
	// 	card_state[i] = false;
	// }

}


function asking(content){

	card_idx = content[0]
	var canvas = document.getElementById('card'+card_idx);
	var ctx=canvas.getContext("2d");
	canvas.width=450;
	canvas.height=600;
	img = cards[card_idx];
	ctx.fillStyle = "yellow";
	ctx.fillRect(0, 0, canvas.width, canvas.height);
	if (card_state[card_idx]){
		ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
	}
	else{
		ctx.drawImage(imgQues, 0, 0, canvas.width, canvas.height);
	}

	document.getElementById("textBox").innerHTML = content[1];

	if (pre_card_idx>=0){

		var canvas = document.getElementById('card'+pre_card_idx);
		var ctx=canvas.getContext("2d");
		canvas.width=300;
		canvas.height=400;

		if (card_state[pre_card_idx]){
			ctx.drawImage(cards[pre_card_idx], 0, 0, canvas.width, canvas.height);
		}
		else{
			ctx.drawImage(imgQues, 0, 0, canvas.width, canvas.height);
		}
	}
	pre_card_idx = content[0];
}


function zoomImage(img_idx){
	var canvas = document.getElementById('card'+img_idx);
	var ctx=canvas.getContext("2d");
	canvas.width=450;
	canvas.height=600;
	
	if(card_state[img_idx]){
		img = cards[img_idx];
	}
	else{
		img = imgQues;
	}

	ctx.drawImage(img, 0, 0,canvas.width, canvas.height);
}


function start() {

	// var card_names = ["apple.png", "banana.png", "cat.png", "dog.png"];
	// var card_id = ["card0", "card1", "card2", "card3"];

	for(i=0;i<card_num;i++){
		card_state[i] = true;
    	var canvas = document.getElementById("card"+i);
    	var ctx = canvas.getContext("2d");
    	ctx.clearRect(0, 0, canvas.width, canvas.height);
    	ctx.drawImage(cards[i], 0, 0, canvas.width, canvas.height);
    }

}

function fold() {

	// var card_id = ["card0", "card1", "card2", "card3"]
	pre_card_idx = -1;
	// if(pre_card_idx >= 0){
	// 	var canvas = document.getElementById('card'+pre_card_idx);
	// 	var ctx=canvas.getContext("2d");
	// 	canvas.width=300;
	// 	canvas.height=400;
	// 	if(card_state[pre_card_idx]){
	// 		img = cards[pre_card_idx];
	// 	}
	// 	else{
	// 		img = imgQues;
	// 	}
	// 	ctx.drawImage(img, 0, 0, img.width, img.height, 0, 0, canvas.width, canvas.height);

	// }

	for(i=0;i<card_num;i++){
		card_state[i] = false;
		var canvas = document.getElementById("card"+i);
		var ctx = canvas.getContext("2d");
		canvas.width=300;
		canvas.height=400;
    	ctx.clearRect(0, 0, canvas.width, canvas.height);
    	ctx.drawImage(imgQues, x=0, y=0, width=canvas.width, height=canvas.height);
    }
    // document.getElementById("textBox").innerHTML = "\t";
}

function webSocket_op() {
    var wsUri = "ws://172.16.0.130:8888/socket";
    websocket = new WebSocket(wsUri);
    websocket.onopen = function(evt) { 
        onOpen(evt);
        };
    websocket.onclose = function(evt) {
        onClose(evt);
    };
    websocket.onmessage = function(evt) {
        onMessage(evt);
    };
    websocket.onerror = function(evt) {
        onError(evt);
    };
    
}
function onOpen(evt) { 
    doSend("CONNECTED!"); 
}  
function onClose(evt) { 
	// alert("DISCONNECTED");
    // writeToScreen("DISCONNECTED");
}  
function onMessage(evt) {
	messageDecode(evt.data);
	// change_img(evt.data);
}  
function onError(evt) { 
    writeToScreen('ERROR: ' + evt.data); 
}  
function doSend(message) { 
    writeToScreen("SENT: " + message);
    websocket.send(message); 
}  
function writeToScreen(message) { 
	console.log(message);
    // var pre = document.createElement("p");
    // pre.style.wordWrap = "break-word";
    // pre.innerHTML = message;
    // output.appendChild(pre); 
}
