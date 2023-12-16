import React from "react";
import { Navbar, Container, Nav, Toast, ToastContainer, Button } from "react-bootstrap";
import { Link } from "react-router-dom";

export default function Header(props) {
  return (
    <React.Fragment>
      <Navbar bg="dark" variant="dark">
        <Container>
          <Navbar.Brand as={Link} to={"/deploy"}>
            UR5 Teaching Webapp
          </Navbar.Brand>
          <Nav className="justify-content-end">
            <Nav.Link as={Link} to={"/"} className="navbar-item">
              Teach
            </Nav.Link>
            <Nav.Link as={Link} to={"/deploy"} className="navbar-item">
              Deploy
            </Nav.Link>
            <Nav.Link as={Link} to={"/mission_control"} className="navbar-item">
              Mission Control
            </Nav.Link>
          </Nav>
          <ToastContainer className="p-3" position="top-end">
            <Toast bg="danger" show={props.showToast}>
              <Toast.Header closeButton={false}>
                <strong className="me-auto">Bootstrap</strong>
              </Toast.Header>
              <Toast.Body>Hello, world! This is a toast message.</Toast.Body>
            </Toast>
          </ToastContainer>
        </Container>
      </Navbar>
    </React.Fragment>
  );
}
