<script>
    export let options;
    export let selectedStore;
    export let active;
    export let name = "Chooser";
    import  "@frc-web-components/fwc/components/sendable-chooser"
</script>
<style>
    select {
    font-size: 3vw;
    background-color: transparent;
    color:white;
    }

    select > option, select::backdrop{
    background-color: black;
    }
</style>
<select style="width:100%; height:100%" value={$selectedStore} on:change={(e)=>selectedStore(e.detail.selected)}>
    {#each options as value}
    <option value={value} key={value}>{value}</option>
    {/each}
</select>
<!-- <frc-sendable-chooser options={options} active={active} default={active} label={name}
    on:change={(e)=>selectedStore(e.detail.selected)}></frc-sendable-chooser> -->
