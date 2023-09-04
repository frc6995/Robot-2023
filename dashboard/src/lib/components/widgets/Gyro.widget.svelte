<script>
    export let data;
    export let name = "Gyro";
    console.log(data)
    export let NT;
    //import  "@frc-web-components/fwc/components/gyro"

    let valueTopic = NT.NTDouble(0.0, data);
    // $: console.log($valueTopic)
</script>

<script context="module" lang="ts">
    import { WidgetDefinition } from "../../ts/elementTypeDefs";
    export let config : WidgetDefinition = {
        "name": "Gyro",
        "id": "gyro",
        "data": {
                type:"string",
                default:"/",
                description:""
            },
        "properties": {

        },
        layout:{
            minHeight: 4,
            minWidth:4
        }
    };
</script>

a
<!-- <frc-gyro style="width:100%; height:100%; background:white;" value={$valueTopic} label={name}></frc-gyro> -->