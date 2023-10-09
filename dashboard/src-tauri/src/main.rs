// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use tauri::{WindowUrl, utils::config::AppUrl, WindowBuilder};


fn main() {
  // let mut port = 0;
  // if cfg!(dev) {
  //   port = 5173;
  // } else {
  //   port = portpicker::pick_unused_port().expect("failed to find unused port");
  // }

  let mut context = tauri::generate_context!();
  // //let url = format!("http://localhost:{}", port).parse().unwrap();
  // let window_url = WindowUrl::External(url);
  // // rewrite the config so the IPC is enabled on this URL
  // context.config_mut().build.dist_dir = AppUrl::Url(window_url.clone());

  tauri::Builder::default()
    .plugin(tauri_plugin_window_state::Builder::default().build())
    .setup(move |app| {
      let operator = WindowBuilder::new(
        app,
        "operator",
        WindowUrl::App("/".into())
      )
      .title("Operator")
      .build()?;
      let main = WindowBuilder::new(
        app,
        "main",
        WindowUrl::App("/main".into())
      )
      .title("Main")
      .build()?;
      Ok(())
    })
    .run(context)
    .expect("error while running tauri application");
}
