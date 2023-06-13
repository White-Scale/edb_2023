<template>
    <el-container>
      <el-header>

      </el-header>

      <el-main>
<!--        <el-card style="width: 40%;" >-->
          <el-form ref="form" :model="form" label-width="80px" label-position="left">
            <el-form-item label="当前经度" style="margin-left: 0px">
              <el-input v-model="form.longitude" placeholder="请输入经度"></el-input>
            </el-form-item>
            <el-form-item label="当前纬度" style="margin-left: 0px">
              <el-input v-model="form.latitude" placeholder="请输入纬度"></el-input>
            </el-form-item>
            <el-form-item label="当前温度" style="margin-left: 0px">
            <el-input
                placeholder="当前温度"
                v-model="form.temperatureoutput"
                :disabled="false">
            </el-input>
            </el-form-item>
              <el-form-item label="当前湿度" style="margin-left: 0px">
                <el-input
                    placeholder="当前湿度"
                    v-model="form.humidityoutput"
                    :disabled="false">
                </el-input>
            </el-form-item>
          </el-form>
          <el-button type="primary" @click="gettemperature" style="position: relative; width: 100px;height: 40px;" >
            获取温度
          </el-button>
<!--        </el-card>-->

      </el-main>
    </el-container>

</template>

<script>
// import request from "@/utils/request"
import router from "@/router/index"
import axios from 'axios'

export default {
  name: "log",
  data() {
    return {
        form:{
          latitude:'',
          longitude:'',
          temperatureoutput:'',
          humidityoutput:''
        }
    }
  },
  methods: {

    gettemperature() {
      axios.get('http://172.20.10.4:5000/getTemp?latitude='+this.form.latitude+'&longitude='+this.form.longitude).then(res => {
        console.log(res)
        if (res.data!=null)
        {
          this.form.temperatureoutput=res.data.temperature;
          this.form.humidityoutput = res.data.humidity;
        }
        else{
          this.$message({
            message: 'No matching position !Please check!',
            type: 'warning'
          });
        }
      }).catch((error) => {
        this.showMessage(error.response);
      })

    },
    getLongitudeLatitude() {
      var _this = this;
      if (navigator.geolocation) {
        navigator.geolocation.getCurrentPosition(
            //locationSuccess 获取成功的话
            function (position) {
              _this.getLongitude = position.coords.longitude;
              _this.getLatitude = position.coords.latitude;
            },
            //locationError  获取失败的话
            function (error) {
              var errorType = [
                "您拒绝共享位置信息",
                "获取不到位置信息",
                "获取位置信息超时",
              ];
              console.log(errorType[error.code - 1]);
            }
        );
      }
    }

  },
}
</script>

<style scoped>
.pict{
  /*background: url("../images/loggin.jpg");*/
  width: 100%;
  height: 100%;
  position: fixed;
  background-size: 100% 100% ;
  overflow: hidden;
}
#particles-js{
  width: 100%;
  height: calc(100% - 100px);
  position: fixed;
}
/*.el-container{*/
/*  text-align: center;*/
/*}*/
.el-main{
  text-align: center;
}
.el-card{
  text-align: center;
  margin-left: 30%;
}
</style>
