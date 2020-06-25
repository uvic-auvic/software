var onboard_sys_info = new ROSLIB.Topic({
    ros : ros,
    name : '/jetson_data_msg',
    messageType : '/monitor/jetson_data_msg'
  });

onboard_sys_info.subscribe(function(message) {
    $('#ram_usage').text(message.RAM_N + " / " + message.RAM_D);
    $('#cpu_1').text(message.CPU_usage_1 + "%");
    $('#cpu_2').text(message.CPU_usage_2+ "%");
    $('#cpu_3').text(message.CPU_usage_3+ "%");
    $('#cpu_4').text(message.CPU_usage_4+ "%");
    $('#cpu_5').text(message.CPU_usage_5+ "%");
    $('#cpu_6').text(message.CPU_usage_6+ "%");
    $('#bcpu_temp').text(message.BCPU_temp + "°");
    $('#mcpu_temp').text(message.MCPU_temp + "°");
    $('#gpu_temp').text(message.GPU_temp+ "°");
    $('#pll_temp').text(message.PLL_temp+ "°");
    $('#tboard_temp').text(message.Tboard_temp+ "°");
    $('#tdiode_temp').text(message.Tdiode_temp+ "°");
    $('#pmic_temp').text(message.PMIC_temp+ "°");
    $('#thermal').text(message.thermal+ "°");
    $('#vdd_in').text(message.VDD_IN_N + " / " + message.VDD_IN_D);
    $('#vdd_cpu').text(message.VDD_CPU_N + " / " + message.VDD_CPU_D);
    $('#vdd_gpu').text(message.VDD_GPU_N + " / " + message.VDD_GPU_D);
    $('#vdd_soc').text(message.VDD_SOC_N + " / " + message.VDD_SOC_D);
    $('#vdd_wifi').text(message.VDD_WIFI_N + " / " + message.VDD_WIFI_D);
    $('#vdd_ddr').text(message.VDD_DDR_N + " / " + message.VDD_DDR_D);
  });
