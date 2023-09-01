<script>
    export let name;
    export let data;
</script>

<!-- svelte-ignore a11y-media-has-caption -->
<iframe src={data} style="width:100%; height:100%" title={name}></iframe>