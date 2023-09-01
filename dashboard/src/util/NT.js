import { NetworkTables, NetworkTablesTypeInfos } from 'ntcore-ts-client';
import { writable } from 'svelte/store';
import { onDestroy } from 'svelte';

class NT {
    constructor(ip) {
        this.ip = ip;

        this.nt  = NetworkTables.getInstanceByURI(ip);
        this.ntSubscribers = {}
    }

    setIP(ip) {
        if(this.ip !== ip) {
            this.nt.changeURI(ip);
        }
        this.ip = ip;        
    }
    NTValue(init, key, topicType) {
        if (this.ntSubscribers[key] !== undefined) {
            return this.ntSubscribers[key];
        }
        console.log("create")
        let _val = init;
        let needsToPublish = false;
        const subs = [];
        let topic = this.nt.createTopic(key, topicType);
        topic.subscribe((value)=>{
                _val = value;
                console.log(value)
                subs.forEach((fn) => fn(_val));
            }
        );

        onDestroy(()=>{
            if (needsToPublish) {
            topic.unpublish();
            }

            topic.unsubscribe();
        })
        
        const subscribe = (cb) => {
            subs.push(cb);
            cb(_val);
        
            return () => {
                const index = subs.findIndex((fn) => fn === cb);
                subs.splice(index, 1);
            };
        };
        
        const set = (v) => {
            if (!needsToPublish) {
                topic.publish();
                needsToPublish = true;
            }
            topic.setValue(v);
            _val = v;
            subs.forEach((fn) => fn(_val));
        };

        const get = () => {
            return _val;
        }
        
        const update = (fn) => set(fn(_val));
        const type = () => topicType;

        // We create our store as a function so that it can be passed as a callback where the value to set is the first parameter
        function store(val) {set(val)}
        store.subscribe = subscribe;
        store.set = set;
        store.get = get;
        store.update = update;
        store.type = type;
        this.ntSubscribers[key] = store;
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