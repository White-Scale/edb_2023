from flask import Flask, jsonify, json, request, render_template
from flask_cors import CORS
# 使用flask的http请求处理模块需要引用 request 的包
app = Flask(__name__)
CORS(app, resources=r'/*',supports_credentials=True)
@app.route('/')
def index():
    return render_template('./index.html')
 
# get方式
@app.route('/getTemp/', methods=['get'])
def get_user():
    # 前端通过get方式提交的请求，用 request.args 和request.values 都可以获取
    # 例如：http://localhost:5000/getTemp?latitude=1.0&longitude=2.0
    print(request.args)
    print(request.values)
    return jsonify({'temperature':'0','humidity':'0'})
 
 

 
 
if __name__ == '__main__':

    app.run('0.0.0.0',port=5000)