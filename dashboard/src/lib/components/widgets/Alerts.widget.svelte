<script lang="ts">
    export let errors : string[] = [];
    export let warnings : string[];
    export let infos : string[];
</script>
<style>
    .container {
        width:100%;
        height: 100%;
        text-align:left;
        padding:5%;
        font-size: 1.5rem;
        display:flex;
        flex-direction: column;
        justify-content: start;
        --timer-bar-color: var(--frc-timer-bar-color, white);
        color: white;
        border:2px solid white;
    }
</style>
<div class="container">
    {#each errors as error}
        <div><span style="color:red">!</span> {error}</div>
    {/each}
    {#each warnings as warning}
    <div>{warning}</div>
    {/each}
    {#each infos as info}
    <div>{info}</div>
    {/each}
</div>