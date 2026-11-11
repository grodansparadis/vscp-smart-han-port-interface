/*
  VSCP droplet alpha webserver

  This file is part of the VSCP (https://www.vscp.org)

  The MIT License (MIT)
  Copyright (C) 2022-2026 Ake Hedman, the VSCP project <info@vscp.org>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  curl 192.168.43.130:80/hello
  curl -X POST --data-binary @registers.txt 192.168.1.112:80/echo > tmpfile
  curl -X PUT -d "0" 192.168.1.112:80/ctrl
  curl -X PUT -d "1" 192.168.1.112:80/ctrl

  1. "curl 192.168.43.130:80/hello"  - tests the GET "\hello" handler
  2. "curl -X POST --data-binary @anyfile 192.168.43.130:80/echo > tmpfile"
      * "anyfile" is the file being sent as request body and "tmpfile" is where the body of the response is saved
      * since the server echoes back the request body, the two files should be same, as can be confirmed using : "cmp
  anyfile tmpfile"
  3. "curl -X PUT -d "0" 192.168.43.130:80/ctrl" - disable /hello and /echo handlers
  4. "curl -X PUT -d "1" 192.168.43.130:80/ctrl" -  enable /hello and /echo handlers

*/

#ifndef __VSCP_ALPHA_WEBSRV_H__
#define __VSCP_ALPHA_WEBSRV_H__

#define WEBPAGE_PARAM_SIZE  128    // Max size of form parameters

// https://www.motobit.com/util/base64-decoder-encoder.asp
// <link href="data:image/x-icon;base64,YourBase64StringHere" rel="icon" type="image/x-icon" />
#define WEBPAGE_FAVICON "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" \
"tFxXFYVCPflIsfrglSq1++4lalmhUI/fW6AOvlKjfvGzH6pEYnpOvnkWZRo6yyrLaG5u5LnnnqGu" \
"ro57doQoLNbZ/36G8Uv+jMwEt2wNgSk41uNwvNehOapRGBa0NNkMnMlxsC1DQQhWl0BPd5ibW24P" \
"7pBXngWLjZ07d7LnxReob9jC+EWH9hPZ/Nr61Sbbm2yQ8PbRLCEUl5KK0gqDHTfYVBUJlCMZTlp0" \
"51qITy0n0fUGb+75NR1th+eeBVcbt3/1DoaH6ujq/AcXpl9FiH6Uktxzm01tlSCT8ek4mWNLVGeJ" \
"Ldi52cYOKeJJyXfu2kTNtoe56aYbOXXwZTbUWGRzI/z3wF7qGjZQXl72xV5GtSvWsKz6l7xW/wBt" \
"bW3EYjEKywdoPZoknkhyYQp2Xz/FqFNBUdVmjgw0MJA+wdbmpdzxyCOUlETo7zrEoU/aSWZ9WNaY" \
"t2VxLW9D3/eJxxPE43FSqWli7Ue50LYHo7KR+x9/GiUlb770WwqccbzIKnY9/CRLl5bQ3n4U3/PZ" \
"tHkz0Wj02gF8friex/6Xfkdp+jPGLmewN9xLedUK+g78gRsbKnn72Dlqb32Ub337wQX///LP2QWG" \
"UsGdyfM8BgYHWd+4CVGzlb8fGWKUKtauW7/ov/93BQA6O2Ic2v83Riem+fpDj7G9pYWpxDSfftpL" \
"WWkZq1bVLXop+R9VbeGtj3wRLQAAACV0RVh0ZGF0ZTpjcmVhdGUAMjAxNy0wOC0yOVQxMDoxMDo1" \
"OSswMDowMJ6/lxIAAAAldEVYdGRhdGU6bW9kaWZ5ADIwMTctMDgtMjlUMTA6MTA6NTkrMDA6MDDv" \
"4i+uAAAARnRFWHRzb2Z0d2FyZQBJbWFnZU1hZ2ljayA2LjcuOC05IDIwMTQtMDUtMTIgUTE2IGh0" \
"dHA6Ly93d3cuaW1hZ2VtYWdpY2sub3Jn3IbtAAAAABh0RVh0VGh1bWI6OkRvY3VtZW50OjpQYWdl" \
"cwAxp/+7LwAAABh0RVh0VGh1bWI6OkltYWdlOjpoZWlnaHQAMTkyDwByhQAAABd0RVh0VGh1bWI6" \
"OkltYWdlOjpXaWR0aAAxOTLTrCEIAAAAGXRFWHRUaHVtYjo6TWltZXR5cGUAaW1hZ2UvcG5nP7JW" \
"TgAAABd0RVh0VGh1bWI6Ok1UaW1lADE1MDQwMDE0NTl00+TtAAAAD3RFWHRUaHVtYjo6U2l6ZQAw" \
"QkKUoj7sAAAAVnRFWHRUaHVtYjo6VVJJAGZpbGU6Ly8vbW50bG9nL2Zhdmljb25zLzIwMTctMDgt" \
"MjkvMDNjODFlNDg1NzQ5OTkwNTJlNmI1M2ZhZjU4ZWUzNTUuaWNvLnBuZ4SbW3QAAAAASUVORK5C" \
"YII="

// https://codebeautify.org/css-beautify-minify
#define WEBPAGE_STYLE_CSS \
"*{box-sizing:border-box}html{background:radial-gradient(circle at top,#1c4655 0%%,#10252f 34%%,#09141b 76%%)}body{margin:0;padding:16px;text-align:center;font-family:Trebuchet MS,Verdana,sans-serif;color:#edf6f7;background:#09141b}p{margin:.8em 0}hr{border:0;border-top:1px solid rgba(255,255,255,.12);margin:16px 0}.app-shell{text-align:left;display:inline-block;width:100%%;max-width:900px;padding:18px;border-radius:18px;background:linear-gradient(180deg,rgba(22,50,64,.96),rgba(13,30,39,.98));border:1px solid rgba(144,210,220,.18)}.app-header{text-align:center;padding:4px 0 14px}.app-title{margin:0;font-size:1.8rem}.app-subtitle{margin:8px 0 0;font-size:1rem;font-weight:700;text-transform:uppercase;color:#f7c66b}.app-footer{text-align:right;font-size:11px;color:#9eb4b8}fieldset{margin:0 0 16px;padding:16px 18px 20px;border-radius:14px;background:rgba(255,255,255,.04);border:1px solid rgba(144,210,220,.12)}legend{padding:0 8px;font-weight:700;color:#f7c66b}form{margin:0}input,select,textarea{width:100%%;padding:12px;border-radius:10px;border:1px solid #c9d7da;background:#edf4f5;color:#11222a;margin:7px 0 14px}input:focus,select:focus,textarea:focus{outline:none;border-color:#6fcfbe}input[type=file]{padding:8px}input[type=checkbox],input[type=radio]{width:1.1em;height:1.1em;margin:0 8px 0 0;vertical-align:-2px;accent-color:#3cbf9e}input[type=range]{width:99%%;padding:0;margin:8px 0 14px;background:transparent}input:disabled,select:disabled,textarea:disabled{background:#798b90!important;color:#495a5f!important;border:2px solid #5f7075;cursor:not-allowed;opacity:1}textarea{resize:vertical;min-height:318px;overflow:auto;background:#e7f0f2;color:#163240;line-height:1.45}button{border:0;border-radius:12px;background:#dfeaea;color:#11222a;min-height:46px;padding:11px 16px;font-size:1rem;font-weight:700;width:100%%;cursor:pointer;margin-top:6px}button:hover{filter:brightness(1.18)}.bred{background:#d96a60;color:#fff}.bgrn{background:#63bf82;color:#082114}.byell{background:#e8bf63;color:#3c2b00}a{color:#8fdde3;text-decoration:none}.p{text-align:left}.q{text-align:right}.hf{display:none}table{width:100%%;border-collapse:collapse;margin:10px 0 18px}td,th{padding:14px 15px;vertical-align:top;border-bottom:1px solid rgba(255,255,255,.06)}tr:last-child td{border-bottom:0}.name{font-size:.9rem;font-weight:700;color:#edf6f7}.prop{font-size:.9rem;color:#9eb4b8}.infoheader{font-size:1rem;font-weight:600;color:#f7c66b}small{display:block;margin-top:9px}@media (max-width:720px){body{padding:12px}.app-shell{padding:14px}.app-title{font-size:1.45rem}table,tbody,tr,td{display:block;width:100%%}td{padding:10px 8px}.app-footer{text-align:center}}"


/*
// https://github.com/Jeija/esp32-softap-ota/blob/master/main/web/index.html
function startUpload() {
				var otafile = document.getElementById("otafile").files;

				if (otafile.length == 0) {
					alert("No file selected!");
				} else {
					document.getElementById("otafile").disabled = true;
					document.getElementById("upload").disabled = true;

					var file = otafile[0];
					var xhr = new XMLHttpRequest();
					xhr.onreadystatechange = function() {
						if (xhr.readyState == 4) {
							if (xhr.status == 200) {
								document.open();
								document.write(xhr.responseText);
								document.close();
							} else if (xhr.status == 0) {
								alert("Server closed the connection abruptly!");
								location.reload()
							} else {
								alert(xhr.status + " Error!\n" + xhr.responseText);
								location.reload()
							}
						}
					};

					xhr.upload.onprogress = function (e) {
						var progress = document.getElementById("progress");
						progress.textContent = "Progress: " + (e.loaded / e.total * 100).toFixed(0) + "%";
					};
					xhr.open("POST", "/update", true);
					xhr.send(file);
				}
			}
*/

/*
#define TTT "function startUpload() { " \
"	var otafile = document.getElementById('otafile').files; " \
" " \
"	if (otafile.length == 0) { " \
"		alert('No file selected!'); " \
"	} else {" \
"		document.getElementById('otafile').disabled = true; " \
"		document.getElementById('upload').disabled = true; " \
" " \
"		var file = otafile[0]; " \
"		var xhr = new XMLHttpRequest(); " \
" " \
"		xhr.onreadystatechange = function() { " \
"			if (xhr.readyState == 4) { " \
"				if (xhr.status == 200) { " \
"					document.open(); " \
"					document.write(xhr.responseText); " \
"					document.close(); " \
"				} else if (xhr.status == 0) { " \
"					alert('Server closed the connection abruptly!'); " \
"					location.reload() " \
"				} else { " \
"					alert(xhr.status + ' Error!' + xhr.responseText); " \
"					location.reload() " \
"				} " \
"			} " \
"		}; " \
" " \
"		xhr.upload.onprogress = function (e) { " \
"			var progress = document.getElementById('progress'); " \
"			progress.textContent = 'Progress: ' + (e.loaded / e.total * 100).toFixed(0) + '%%'; " \
"		}; " \
" } " \
" alert(\"hello\");" \
"}"
*/


#define WEBPAGE_JS1 "function startUpload(){var e,t=document.getElementById(\"otafile\").files;0==t.length?alert(\"No file selected!\"):(document.getElementById(\"otafile\").disabled=!0,document.getElementById(\"upload\").disabled=!0,t=t[0],(e=new XMLHttpRequest).onreadystatechange=function(){4==e.readyState&&(200==e.status?(document.open(),document.write(e.responseText),document.close()):(0==e.status?alert(\"Server closed the connection abruptly!\"):alert(e.status+\" Error!\"+e.responseText),location.reload()))},e.upload.onprogress=function(e){document.getElementById(\"progress\").textContent=\"Progress: \"+(e.loaded/e.total*100).toFixed(0)+\"%%\"},e.open(\"POST\",\"/upgrdlocal\",!0),e.send(t))}"
#define WEBPAGE_JS2 "function startUploadSibLocal(){var e,t=document.getElementById(\"otafile_sib\").files;0==t.length?alert(\"No file selected!\"):(document.getElementById(\"otafile_sib\").disabled=!0,document.getElementById(\"upload_sib\").disabled=!0,t=t[0],(e=new XMLHttpRequest).onreadystatechange=function(){4==e.readyState&&(200==e.status?(document.open(),document.write(e.responseText),document.close()):(0==e.status?alert(\"Server closed the connection abruptly!\"):alert(e.status+\" Error!\"+e.responseText),location.reload()))},e.upload.onprogress=function(e){document.getElementById(\"progress_sib\").textContent=\"Progress: \"+(e.loaded/e.total*100).toFixed(0)+\"%%\"},e.open(\"POST\",\"/upgrdSiblingLocal\",!0),e.send(t))}"

/*>>
  Page start HTML
  Parameter 1: Page head
  Parameter 2: Section header  
  "<link rel=\"stylesheet\" href=\"style.css\" /></head><body><div " \
  "<link rel=\"icon\" href=\"favicon.ico\">" \
*/
#define WEBPAGE_START_TEMPLATE "<!DOCTYPE html><html lang=\"en\"><head><meta charset='utf-8'>" \
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,user-scalable=no\" />" \
"<title>VSCP Gateway</title>"\
"<script>" \
WEBPAGE_JS1 \
WEBPAGE_JS2 \
"</script>" \
"<style>" \
WEBPAGE_STYLE_CSS \
"</style>" \
"<link href=\"data:image/x-icon;base64," \
WEBPAGE_FAVICON \
"\" rel=\"icon\" type=\"image/x-icon\" />" \
"</head><body><div class='app-shell'>" \
"<div class='app-header'>" \
"<h3 class='app-title'>%s</h3>" \
"<h4 class='app-subtitle'>%s</h4></div>"

/*>>
  Page end HTML
  Parameter 1: Page head
  Parameter 2: Section header
*/
#define WEBPAGE_END_TEMPLATE "<div class='app-footer'><hr />"\
"<form id=but14 style=\"display: block;\" "\
"action='index.html' method='get'><button class=\"byell\">Main Menu</button></form>"\
"<hr /><div>%s - <a href='https://vscp.org' target='_blank'>%s -- vscp.org</a></div>"\
"</div></body></html>"

#define WEBPAGE_CONFIG_END_TEMPLATE "<div class='app-footer'><hr />"\
"<form id=but14 style=\"display: block;\" "\
"action='config.html' method='get'><button class=\"byell\">Configuration</button></form>"\
"<hr /><div>%s - <a href='https://vscp.org' target='_blank'>%s -- vscp.org</a></div>"\
"</div></body></html>"

#define WEBPAGE_END_TEMPLATE_NO_RETURN "<div class='app-footer'>"\
"<hr /><div>%s - <a href='https://vscp.org' target='_blank'>%s -- vscp.org</a></div>"\
"</div></body></html>"

#define CONFIG_DEFAULT_BASIC_AUTH_USERNAME "vscp"
#define CONFIG_DEFAULT_BASIC_AUTH_PASSWORD "secret"

#define HTTPD_401 "401 UNAUTHORIZED" /*!< HTTP Response 401 */

typedef struct {
  char *username;
  char *password;
} basic_auth_info_t;

/*!
  Start the webserver
  @return esp error code
*/

httpd_handle_t
start_webserver(void);

/*!
  Stop the webserver
  @param server Server handle
  @return esp error code
*/

esp_err_t
stop_webserver(httpd_handle_t server);

#endif