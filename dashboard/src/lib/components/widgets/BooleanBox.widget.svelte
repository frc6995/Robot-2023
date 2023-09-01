<script>
    export let value;
    export let label;
</script>
<div style={`width:100%; height: 100%; background-color:${value? "green" : "red"}`}><div style="margin:auto">{label}</div></div>