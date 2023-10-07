import { NetworkTables, NetworkTablesTypeInfos } from 'ntcore-ts-client';
import { writable } from 'svelte/store';
import { onDestroy } from 'svelte';

class NT {
    constructor(ip) {
        this.ip = ip;
        
        this.nt  = NetworkTables.getInstanceByURI(ip);
        this.nt.changeURI("10.69.95.2");
        this.ntSubscribers = {}
    }

    setIP(ip) {
        if(this.ip !== ip) {
            this.nt.changeURI(ip);
        }
        this.ip = ip;        
    }
    NTValue(init, key, topicType) {

        const internal = writable(init);
        console.log("create")
        let _val = init;
        let isPublishing = false;
        const subs = [];
        let topic = this.nt.createTopic(key, topicType);
        let subuuid = topic.subscribe((value)=>{
                internal.set(value);
            }
        );
       

        const subscribe = internal.subscribe;
        
        const set = (v) => {
            if (!topic.publisher) {
                topic.publish();
            }
            topic.setValue(v);
            internal.set(v);
        };

        const get = ()=>topic.getValue();
        
        const update = (fn) => set(fn(_val));
        const type = () => topicType;

        // We create our store as a function so that it can be passed as a callback where the value to set is the first parameter
        function store(val) {set(val)}
        store.subscribe = subscribe;
        store.set = set;
        store.get = get;
        store.update = update;
        store.type = type;
        return store;
    }

    NTInt(init, key){return this.NTValue(init, key, NetworkTablesTypeInfos.kInteger) }
    NTDouble(init, key){return this.NTValue(init, key, NetworkTablesTypeInfos.kDouble) }
    NTBoolean(init, key){return this.NTValue(init, key, NetworkTablesTypeInfos.kBoolean) }
    NTString(init, key){return this.NTValue(init, key, NetworkTablesTypeInfos.kString) }
    NTIntArray(init, key){return this.NTValue(init, key, NetworkTablesTypeInfos.kIntegerArray) }
    NTDoubleArray(init, key){return this.NTValue(init, key, NetworkTablesTypeInfos.kDoubleArray) }
    NTBooleanArray(init, key){return this.NTValue(init, key, NetworkTablesTypeInfos.kBooleanArray) }
    NTStringArray(init, key){return this.NTValue(init, key, NetworkTablesTypeInfos.kStringArray) }
    
}




export default new NT("127.0.0.1");