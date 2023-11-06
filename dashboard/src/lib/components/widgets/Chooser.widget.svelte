<script>
    export let options;
    export let selectedStore;
    export let active;
    export let disabled;
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
<select style="width:100%; height:100%" disabled={disabled} value={active} on:change={(e)=>{
    console.log(e)
    selectedStore(e.target.value)}}>
    {#each options as value}
    <option value={value} key={value}>{value}</option>
    {/each}
</select>
<!-- <frc-sendable-chooser options={options} active={active} default={active} label={name}
    on:change={(e)=>selectedStore(e.detail.selected)}></frc-sendable-chooser> -->
