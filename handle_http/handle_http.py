from flask import Flask, jsonify, json, request, render_template
from flask_cors import CORS
import serial
import time
import _thread 
import numpy as np
from scipy.spatial import Delaunay
# 串口部分
node_num = [0] * 10
node_tempature = [0] * 10
node_humidity = [0] * 10
node_points = [[0, 0]] * 10


def ReadSerialPort(port='COM3'):
    com = serial.Serial(port, 115200)
    if com.isOpen():
        print("open success")
    while True:
        line = com.readline()
        if line.__len__()>4 or line.__len__() < 3  :
            continue
        num = int(line.decode().strip())
        for i in range(num):
            line2 = com.readline().decode().strip()

            num, tempature, humidity, x, y = line2.split()
            node_num[i], node_humidity[i], node_tempature[i], = int(
                num), int(tempature), int(humidity)
            node_points[i] = [int(x), int(y)]
            # print(node_num[i], node_tempature[i], node_humidity[i], node_points[i])
        time.sleep(1)

def GetTHByLocation(latitude: float, longitude: float) -> tuple([float, float]):
    tri = Delaunay(node_points)
    # print(tri.simplices)
    p = [latitude, longitude]  # search
    p1 = node_points[tri.simplices[tri.find_simplex(p)][0]]
    p2 = node_points[tri.simplices[tri.find_simplex(p)][1]]
    p3 = node_points[tri.simplices[tri.find_simplex(p)][2]]
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)

    v1v2 = p2 - p1
    v1v3 = p3 - p1
    v1p = p - p1
    d00 = np.dot(v1v2, v1v2)
    d01 = np.dot(v1v2, v1v3)
    d11 = np.dot(v1v3, v1v3)
    d20 = np.dot(v1p, v1v2)
    d21 = np.dot(v1p, v1v3)
    denom = d00 * d11 - d01 * d01
    u = (d11 * d20 - d01 * d21) / denom
    v = (d00 * d21 - d01 * d20) / denom
    w = 1 - u - v

    # Interpolate value
    h1 = node_humidity[tri.simplices[tri.find_simplex(p)][0]]
    h2 = node_humidity[tri.simplices[tri.find_simplex(p)][1]]
    h3 = node_humidity[tri.simplices[tri.find_simplex(p)][2]]

    t1 = node_tempature[tri.simplices[tri.find_simplex(p)][0]]
    t2 = node_tempature[tri.simplices[tri.find_simplex(p)][1]]
    t3 = node_tempature[tri.simplices[tri.find_simplex(p)][2]]

    H = u * h1 + v * h2 + w * h3
    T = u * t1 + v * t2 + w * t3
    print("H = ", H)
    print("T = ", T)
    return H, T


# 使用flask的http请求处理模块需要引用 request 的包
app = Flask(__name__)
CORS(app, resources=r'/*', supports_credentials=True)


@app.route('/')
def index():
    return render_template('./index.html')

# get方式


@app.route('/getTemp/', methods=['get'])
def get_user():
    # 前端通过get方式提交的请求，用 request.args 和request.values 都可以获取
    # 例如：http://localhost:5000/getTemp?latitude=1.0&longitude=2.0
    # print(request.args)
    print(request.values)
    try:
        # print(request.values.get('latitude'), request.values.get('longitude'))
        H, T = GetTHByLocation(
            float(request.values.get('latitude')), float(request.values.get('longitude')))
    except:
        print("serial port read error! set H T to default")
        H, T = 0, 0

    return jsonify({'temperature': H, 'humidity': T})


if __name__ == '__main__':
    _thread.start_new_thread (ReadSerialPort)
    app.run('0.0.0.0', port=5000)
