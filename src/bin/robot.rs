struct Robot {
    name: String,
}

impl Robot {
    fn init(&self) {
        println!("{} is initialized", self.name);
    }
    fn observe(&self) {
        println!("{} is observing", self.name);
        std::thread::sleep(std::time::Duration::from_millis(800));
    }
    fn planning(&self) {
        println!("{} is planning", self.name);
        std::thread::sleep(std::time::Duration::from_millis(300));
    }
    fn execute(&self) {
        println!("{} is executing", self.name);
        std::thread::sleep(std::time::Duration::from_millis(1500));
    }
    fn step(&self) {
        self.observe();
        self.planning();
        self.execute();
    }
}

fn main() {
    let robot = Robot {
        name: "Rocky One".to_string(),
    };

    robot.init();
    loop {
        robot.step();
    }
}
