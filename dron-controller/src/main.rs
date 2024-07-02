use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use std::net::UdpSocket;
use core::net::Ipv4Addr;
use core::time::Duration;
use std::str::FromStr;
use core::net::SocketAddrV4;
use bevy::input::gamepad::GamepadAxisChangedEvent;
use bevy::math::Quat;
use bevy::input::gamepad::GamepadButtonChangedEvent;
use gilrs::{Gilrs, Button, Event};
use std::sync::Mutex;
use gilrs::EventType;
use crate::egui::DragValue;

#[derive(Debug, PartialEq)]
enum State {
    None,
    Init,
    Started,
    Booted,
}

#[repr(u8)]
enum TXMsg {
    IsConnected = 0,
    BootOn      = 1,
    BootOff     = 2,
    Command     = 3,
}

#[derive(Component)]
struct Controller {
    wheneblock: u8,
    boot:       bool,
    state:      State,
    command:    [i8;3],
    socket:     UdpSocket,
    addr:       Option<SocketAddrV4>
}

#[derive(Component,Debug)]
struct Buffer(String,String,u16,f32,f32,f32);

#[derive(Resource)]
struct EvHand {
    hand:   Mutex<Gilrs>,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin)
        // Systems that create Egui widgets should be run during the `CoreSet::Update` set,
        // or after the `EguiSet::BeginFrame` system (which belongs to the `CoreSet::PreUpdate` set).
        .add_systems(Startup, init)
        .add_systems(Update, ui_system)
        .add_systems(Update, (update_axes, update_button,update_button_test,sync_state))
        .run();
}

fn sync_state(mut query:  Query<(&mut Controller,&mut Transform)>,buffer: Query<&mut Buffer>) {
    let buffer = buffer.get_single().unwrap();
    if let Ok((mut c,mut t)) = query.get_single_mut() {
        if c.addr.is_none() { return () }
        if c.wheneblock == 0 {
            let mut TX_BUFF = [0;18];
            let f = buffer.2.to_le_bytes();
            let am = buffer.4.to_le_bytes();
            let ap = buffer.5.to_le_bytes();
            let m = buffer.3.to_le_bytes();
            for i in 0..4  {
                TX_BUFF[4 + i] = m[i];
            }
            for i in 0..4  {
                TX_BUFF[8 + i] = am[i];
            }
            for i in 0..4  {
                TX_BUFF[12 + i] = ap[i];
            }
            for i in 0..2  {
                TX_BUFF[16 + i] = f[i];
            }
            if c.boot {
                TX_BUFF[0] = TXMsg::Command as u8;
                // set commands
                TX_BUFF[1] = c.command[0].to_be_bytes()[0];
                TX_BUFF[2] = c.command[1].to_be_bytes()[0];
                TX_BUFF[3] = c.command[2].to_be_bytes()[0];
            }
            //println!("{:?}",TX_BUFF);
            match c.state {
                State::Booted => {
                    if !c.boot  {
                        TX_BUFF[0] = TXMsg::BootOff as u8;
                    }
                },
                State::Started if c.boot => TX_BUFF[0] = TXMsg::BootOn as u8,
                _   => TX_BUFF[0] = TXMsg::IsConnected as u8,
            }
            let r = c.socket.send_to(&TX_BUFF,c.addr.unwrap());
            let mut buf = [0; 13];
            match c.socket.recv_from(&mut buf) {
                Ok(r)   => {
                    match buf {
                        [0x00, ..]          => c.state = State::Init,
                        [0x01, ..]          => c.state = State::Started,
                        [0x02, rest @ ..]   => { 
                            c.state = State::Booted;
                            c.boot  = true;
                            let z = f32::from_ne_bytes(rest[..4].try_into().unwrap());
                            let y = f32::from_ne_bytes(rest[4..8].try_into().unwrap());
                            let x = f32::from_ne_bytes(rest[8..].try_into().unwrap());
                            t.rotation = Quat::from_scaled_axis([-1.0 * y,-1.0 * z,x].into());
                            //println!("x: {},y: {},z: {}",x,y,z);
                        },
                        _ => (),
                    }
                },
                Err(e)  => {
                    c.wheneblock = 2;
                },
            }
        } else {
            c.wheneblock -= 1;
        }
    }
}
fn update_button(
    button_inputs: Res<ButtonInput<GamepadButton>>,
    mut query:  Query<&mut Controller>,
) {
    let pad = Gamepad::new(0);
    if let Ok(mut c) = query.get_single_mut() {
        if button_inputs.just_pressed(GamepadButton::new(pad, GamepadButtonType::DPadUp)) { c.command[1] = 127; }
        if button_inputs.just_released(GamepadButton::new(pad, GamepadButtonType::DPadUp)) { c.command[1] = 0; }
        if button_inputs.just_pressed(GamepadButton::new(pad, GamepadButtonType::DPadDown)) { c.command[1] = -128; }
        if button_inputs.just_released(GamepadButton::new(pad, GamepadButtonType::DPadDown)) { c.command[1] = 0; }
        if button_inputs.just_pressed(GamepadButton::new(pad, GamepadButtonType::DPadLeft)) { c.command[0] = -128; }
        if button_inputs.just_released(GamepadButton::new(pad, GamepadButtonType::DPadLeft)) { c.command[0] = 0; }
        if button_inputs.just_pressed(GamepadButton::new(pad, GamepadButtonType::DPadRight)) { c.command[0] = 127; }
        if button_inputs.just_released(GamepadButton::new(pad, GamepadButtonType::DPadRight)) { c.command[0] = 0; }

    }
}

fn update_button_test(mut query: Query<&mut Controller>,ev_hand: Res<EvHand>) {
    if let Ok(mut c) = query.get_single_mut() {
        while let Some(Event { id, event, time }) = ev_hand.hand.lock().unwrap().next_event() {
            match event {
                EventType::ButtonPressed(Button::Unknown, code ) if code.into_u32() == 65833 => {
                    println!("boooot");
                    c.wheneblock = 0;
                    c.boot = !c.boot;
                },
                _ => (),
            }
        }
    }
}

fn update_axes(mut axis_events: EventReader<GamepadAxisChangedEvent>, mut query: Query<&mut Controller>) {
    for axis_event in axis_events.read() {
        let axis_type = axis_event.axis_type;
        let value = axis_event.value;
        if let Ok(mut c) = query.get_single_mut() {
            match axis_type {
            GamepadAxisType::LeftStickX     => c.command[0] = (value * 128.0) as i8,
            GamepadAxisType::LeftStickY     => c.command[1] = (value * 128.0) as i8,
            GamepadAxisType::RightStickX    => c.command[2] = (value * 128.0) as i8,
            _ =>(),
            }
        }
    }
}

fn init(mut commands: Commands, asset_server: Res<AssetServer>) {
    let socket = UdpSocket::bind("0.0.0.0:0").unwrap();
    socket.set_nonblocking(true).unwrap();

    let h = Mutex::new(Gilrs::new().unwrap());

    commands.insert_resource(EvHand { hand: h });
    commands.spawn((Controller {
        wheneblock: 0,
        boot:       false,
        state:      State::None,
        command:    [0;3],
        socket,
        addr:       None 
    },
    SceneBundle {
        scene: asset_server.load("untitled_1.glb#Scene0"),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    }));
    commands.spawn(Buffer(String::new(),String::new(),300,0.1,6.0,0.12247));
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-5.0, 2.0, 0.0)
            .looking_at(0.0 * Vec3::Y, Vec3::Y),
            ..Default::default()
        });
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity:          600_000.0,
            shadows_enabled:    false,
            ..default()
        },
        transform: Transform::from_xyz(0.0, 5.0, 0.0),
        ..default()
    });
}

fn ui_system(mut contexts: EguiContexts, mut query: Query<&mut Controller>, mut buffer: Query<&mut Buffer>) {
    egui::Window::new("socket input").show(contexts.ctx_mut(), |ui| {
        ui.vertical(|ui| {
            if let (Ok(mut ctr),Ok(mut buffer)) = (query.get_single_mut(), buffer.get_single_mut()) {
                ui.label(format!("state: {:?}", ctr.state));
                ui.text_edit_singleline(&mut buffer.0);
                if ui.button("Submet").clicked() {
                    
                     match Ipv4Addr::from_str(buffer.0.as_str()) {
                        Ok(r)   => {
                            buffer.1 = format!("valid address: {}",r);
                            let addr = SocketAddrV4::new(r, 1234);
                            ctr.addr = Some(addr);
                        }
                        Err(e)  =>{ 
                            buffer.1 = format!("invalid address: {:?}",e);
                        }
                    }
                }
                ui.label(buffer.1.clone());
                ui.horizontal(|ui| {
                    ui.label("force Thrust");
                    ui.add(DragValue::new(&mut buffer.2));
                });
                ui.horizontal(|ui| {
                    ui.label("motor mass");
                    ui.add(DragValue::new(&mut buffer.3).speed(0.001).clamp_range(0.075..=0.12));
                });
                ui.horizontal(|ui| {
                    ui.label("coefficient d'amortissement");
                    ui.add(DragValue::new(&mut buffer.4).speed(0.005).clamp_range(0..=6));
                });
                ui.horizontal(|ui| {
                    ui.label("La pulsation propre ");
                    ui.add(DragValue::new(&mut buffer.5).speed(0.001).clamp_range(0.0..=0.5));
                });
            }
        });
    });
}


